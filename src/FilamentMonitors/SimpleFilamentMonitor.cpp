/*
 * SimpleFilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "SimpleFilamentMonitor.h"
#include <Heating/Heat.h>
#include <Tools/Tool.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Movement/Move.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(SimpleFilamentMonitor, __VA_ARGS__)

constexpr ObjectModelTableEntry SimpleFilamentMonitor::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	{ "enabled",				OBJECT_MODEL_FUNC(self->enabled),		 					ObjectModelEntryFlags::none },
	{ "extruderEnergyPerSec",	OBJECT_MODEL_FUNC(self->extruderEnergyPerSec, 3),			ObjectModelEntryFlags::live },
	{ "extrusionLastSegment",	OBJECT_MODEL_FUNC(self->extrusionLastSegment, 3),			ObjectModelEntryFlags::live },
	{ "extrusionLength",		OBJECT_MODEL_FUNC(self->extrusionCommandedThisSegment, 3),	ObjectModelEntryFlags::live },
	{ "failCount",				OBJECT_MODEL_FUNC(self->failCount, 1),						ObjectModelEntryFlags::live },
	{ "heaterEnergyPerSec",		OBJECT_MODEL_FUNC(self->heaterEnergyPerSec, 3),				ObjectModelEntryFlags::live },
	{ "status",					OBJECT_MODEL_FUNC(self->GetStatusText()),					ObjectModelEntryFlags::live },
	{ "sumExtruderEnergy",		OBJECT_MODEL_FUNC(self->sumExtruderEnergy, 3),				ObjectModelEntryFlags::live },
	{ "sumHeaterEnergy",		OBJECT_MODEL_FUNC(self->sumHeaterEnergy, 3),				ObjectModelEntryFlags::live },
	{ "type",					OBJECT_MODEL_FUNC_NOSELF("simple"), 						ObjectModelEntryFlags::none },

};

constexpr uint8_t SimpleFilamentMonitor::objectModelTableDescriptor[] = { 1, 10};

DEFINE_GET_OBJECT_MODEL_TABLE(SimpleFilamentMonitor)

#endif

SimpleFilamentMonitor::SimpleFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept
	: FilamentMonitor(drv, monitorType, did),
	  highWhenNoFilament(monitorType == 2),
	  filamentPresent(false),
	  lastFanState(false),
	  enabled(false),
	  deadTime(0.312),
	  deadTimeBufferIndex(0),
	  extrusionCommandedThisSegment(0.0),
	  sumHeaterEnergy(0.0),
	  sumExtruderEnergy(0.0),
	  minEnergyConsumption(25.0),
	  heaterEnergyPerSec(0.0),
	  extruderEnergyPerSec(0.0),
	  heaterNumber(0),
	  failCount(0),
	  extrusionStartingTime(0),
	  lastSegmentTime(0),
	  lastControlTime(0),
	  lastCheckTime(0),
	  checkDelay(5000),
	  additinalOnetimeDelay(0),
	  extruder(LogicalDriveToExtruder(drv))
{
	for( unsigned int i=0; i < (sizeof(deadTimeBuffer)/sizeof(deadTimeBuffer[0])); i++)
	{
		deadTimeBuffer[i] = 0;
	}
}

int SimpleFilamentMonitor::GetToolNumberForDrive() noexcept
{
	// Get assigned heater of extruder drive over tool linking
	// TBD: allow more than one tool per extruder / extruder per tool
	for (unsigned int tNumber = 0; tNumber < reprap.GetNumberOfContiguousTools(); tNumber++)
	{
		ReadLockedPointer<Tool> tool = reprap.GetTool(tNumber);
		if (tool.IsNull())
		{
			continue;
		}

		for (unsigned int driveNum = 0; driveNum < tool->DriveCount(); driveNum++)
		{
			const unsigned int extruderDrive = (unsigned int)(tool->GetDrive(driveNum));
			if (extruderDrive == extruder)
			{
				return tNumber;
			}
		}
	}
	return -1;
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult SimpleFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException)
{
	const GCodeResult rslt = CommonConfigure(gb, reply, InterruptMode::none, seen);
	if (Succeeded(rslt))
	{
		// Raa : aa in percent of underpower
		// Enn : minimum extrusion time
		// An : 1 = check all extruder moves ; 0 = only printing
		// Bnn : base value for avgPwm without fan in idle at desired temp
		// Lnn : calibration factor / slope

		ReadLockedPointer<Tool> tool = reprap.GetTool(GetToolNumberForDrive());
		if (tool.IsNull())
		{
			throw GCodeException(-1, -1, "No tool number given and no current tool");
		}
		heaterNumber = tool->GetHeater(0);


		if (gb.Seen('S'))
		{
			seen = true;
			enabled = (gb.GetIValue() > 0);
		}

		if (seen)
		{
			Check(false, false, 0, 0.0);
			reprap.SensorsUpdated();
		}
		else
		{
			reply.copy("Simple filament sensor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, output %s when no filament, filament present: %s",
						(enabled) ? "enabled" : "disabled",
						(highWhenNoFilament) ? "high" : "low",
						(filamentPresent) ? "yes" : "no");
		}
	}
	return rslt;
}

// ISR for when the pin state changes
bool SimpleFilamentMonitor::Interrupt() noexcept
{
	// Nothing needed here
	GetPort().DetachInterrupt();
	return false;
}

// Call the following regularly to keep the status up to date
void SimpleFilamentMonitor::Poll() noexcept
{
	const bool b = GetPort().ReadDigital();
	filamentPresent = (highWhenNoFilament) ? !b : b;
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
FilamentSensorStatus SimpleFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept
{
	Poll();

	return CheckFilament(isPrinting, fromIsr, isrMillis, filamentConsumed);
}

float SimpleFilamentMonitor::GetFanSpeed() noexcept
{
	const Tool * const ct = reprap.GetCurrentTool();
	if (ct != nullptr)
	{
		FansBitmap fanMapping = ct->GetFanMapping();
		for (size_t fi = 0; fi < MaxFans; ++fi)
		{
			if (fanMapping.IsBitSet(fi))
			{
				return reprap.GetFansManager().GetFanValue(fi);
			}
		}
	}
	return -2.0;
}

FilamentSensorStatus SimpleFilamentMonitor::CheckFilament(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept
{
	const uint32_t now = millis();
	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	float currentValue = reprap.GetHeat().GetAveragePWM((size_t)heaterNumber);
	bool fanState = false;
	float fanSpeed = GetFanSpeed();
	fanState = fanSpeed>0?true:false;

	if(filamentConsumed != 0.0) {
		if( extrusionStartingTime == 0)
		{
			extrusionStartingTime = lastSegmentTime;
			extrusionEndTime = now;
		}
		else
		{
			extrusionEndTime = now;
		}

		extrusionLastSegment += filamentConsumed;
	}

	float timeDelaySec = (now - lastControlTime)/1000.0;
	float surfaceArea = 0.06751509;
	float deltaT = (reprap.GetHeat().GetHeaterTemperature(heaterNumber) - 20.0);
	float totalHeatLosses = ((0.00157*deltaT+0.68033)*deltaT*surfaceArea);
	float totalFanLosses = 0.0;

	if(fanState) {
		totalFanLosses = (-0.00087*deltaT+1.20034)*deltaT*surfaceArea + ((-0.00017*(float)pow(fanSpeed*100.0, 2.0)) + 3.415/*0.03415*100*/ * fanSpeed - 1.71875);
	}

	heaterEnergyPerSec = currentValue * 50.0 - totalHeatLosses - totalFanLosses; // 50 Watt heater -> 1W = 1 J/s
	if( (float)fabs(heaterEnergyPerSec) < 0.1 )
	{
		heaterEnergyPerSec = 0;
	}
	sumHeaterEnergy += (heaterEnergyPerSec * (now - lastSegmentTime))/1000.0;

	if(timeDelaySec > deadTime)
	{
		lastControlTime = now;
		float volume = 6.3793966 * extrusionLastSegment; // PI * pow(d, 2)/4 * length
		float mass = volume * 1.27/1000000.0; // cm³ -> mm³ / g -> kg
		float energyPerSec = (1500.0 * mass * deltaT)/timeDelaySec; // J/(kg*K) /
		deadTimeBuffer[deadTimeBufferIndex] = energyPerSec;
		extrusionLastSegment = 0.0;
		extrusionStartingTime = 0;
		deadTimeBufferIndex = (deadTimeBufferIndex+1)%(sizeof(deadTimeBuffer)/sizeof(deadTimeBuffer[0]));
		// before that line is the presents

		// here is the past

		sumExtruderEnergy += deadTimeBuffer[deadTimeBufferIndex]*deadTime;
		extruderEnergyPerSec = deadTimeBuffer[deadTimeBufferIndex];
	}

	if(lastFanState != fanState)
	{
		if( fanState )
		{
			additinalOnetimeDelay = checkDelay * 2;
		}
	}

	if((now - lastCheckTime) > (checkDelay + additinalOnetimeDelay))
	{
		additinalOnetimeDelay = 0;
		lastCheckTime = now;
		if( sumExtruderEnergy > minEnergyConsumption )
		{
			if( sumHeaterEnergy < sumExtruderEnergy )
			{
				if(failCount < 3)
					failCount++;
			}
			else
			{
				if(failCount > 0)
					failCount--;
			}
			sumHeaterEnergy = 0;
			sumExtruderEnergy = 0;
		}
	}

	if( failCount >= 3 )
	{
		failCount = 0;
		ret = FilamentSensorStatus::tooLittleMovement;
	}
	else if (filamentPresent == false)
	{
		ret = FilamentSensorStatus::noFilament;
	}

	lastSegmentTime = now;
	lastFanState = fanState;

	return ret;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus SimpleFilamentMonitor::Clear() noexcept
{
	Poll();
	bool isPrinting = false;
	FilamentSensorStatus ret = FilamentSensorStatus::ok;

	int32_t extruderStepsCommanded = reprap.GetMove().GetAccumulatedExtrusion(ExtruderToLogicalDrive(extruder), isPrinting);
	// is the extruder moving filament forward?
	const float filamentConsumed = (float)extruderStepsCommanded/reprap.GetPlatform().DriveStepsPerUnit(ExtruderToLogicalDrive(extruder));

	CheckFilament(isPrinting, false, 0, filamentConsumed);

	return ret;
}

// Print diagnostic info for this sensor
void SimpleFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder) noexcept
{
	Poll();
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: %s\n", extruder, (filamentPresent) ? "ok" : "no filament");
}

#if SUPPORT_REMOTE_COMMANDS

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult SimpleFilamentMonitor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	const GCodeResult rslt = CommonConfigure(parser, reply, InterruptMode::none, seen);
	if (rslt <= GCodeResult::warning)
	{
		uint16_t temp;
		if (parser.GetUintParam('S', temp))
		{
			seen = true;
			enabled = (temp > 0);
		}


		if (seen)
		{
			Check(false, false, 0, 0.0);
		}
		else
		{
			reply.copy("Simple filament sensor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, output %s when no filament, filament present: %s",
						(enabled) ? "enabled" : "disabled",
						(highWhenNoFilament) ? "high" : "low",
						(filamentPresent) ? "yes" : "no");
		}
	}
	return rslt;
}

#endif

// End
