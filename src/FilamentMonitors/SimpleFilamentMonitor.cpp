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
	{ "baseValue",			OBJECT_MODEL_FUNC(self->baseValue, 3),		 			ObjectModelEntryFlags::live },
	{ "currentValue",		OBJECT_MODEL_FUNC(self->currentValue, 3),		 		ObjectModelEntryFlags::live },
	{ "enabled",			OBJECT_MODEL_FUNC(self->enabled),		 				ObjectModelEntryFlags::none },
	{ "expectedValue",		OBJECT_MODEL_FUNC(self->expectedValue, 3),		 		ObjectModelEntryFlags::live },
	{ "extrusionLastSegment",	OBJECT_MODEL_FUNC(self->extrusionLastSegment, 3),	ObjectModelEntryFlags::live },
	{ "extrusionLength",	OBJECT_MODEL_FUNC(self->extrusionCommandedThisSegment, 3),	ObjectModelEntryFlags::live },
	//{ "extrusionStartingTime",	OBJECT_MODEL_FUNC(self->extrusionStartingTime),	ObjectModelEntryFlags::live },
	{ "failCounter",		OBJECT_MODEL_FUNC(self->failCounter,1),					ObjectModelEntryFlags::live },
	//{ "lastSegmentTime",			OBJECT_MODEL_FUNC(self->lastSegmentTime),						ObjectModelEntryFlags::live },
	{ "lastValue",			OBJECT_MODEL_FUNC(self->lastValue, 3),						ObjectModelEntryFlags::live },
	{ "maxBaseValue",		OBJECT_MODEL_FUNC(self->maxBaseValue, 3),						ObjectModelEntryFlags::live },
	{ "minBaseValue",		OBJECT_MODEL_FUNC(self->minBaseValue, 3),						ObjectModelEntryFlags::live },
	{ "mmPerSec",			OBJECT_MODEL_FUNC(self->mmPerSec, 3),						ObjectModelEntryFlags::live },
	{ "status",				OBJECT_MODEL_FUNC(self->GetStatusText()),				ObjectModelEntryFlags::live },
	{ "steadyCounter",		OBJECT_MODEL_FUNC(self->steadyCounter,1),					ObjectModelEntryFlags::live },
	{ "steadySinceTime",	OBJECT_MODEL_FUNC(self->steadySinceTime),				ObjectModelEntryFlags::live },
	{ "type",				OBJECT_MODEL_FUNC_NOSELF("simple"), 					ObjectModelEntryFlags::none },
};

constexpr uint8_t SimpleFilamentMonitor::objectModelTableDescriptor[] = { 1, 15 };

DEFINE_GET_OBJECT_MODEL_TABLE(SimpleFilamentMonitor)

#endif

SimpleFilamentMonitor::SimpleFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept
	: FilamentMonitor(drv, monitorType, did),
	  highWhenNoFilament(monitorType == 2),
	  filamentPresent(false),
	  enabled(false),
	  baseValue(0.24),
	  currentValue(0.0),
	  lastValue(0.0),
	  expectedValue(0.0),
	  mmPerSec(0.0),
	  fanOffset(0.28),
	  slope(0.0495),
	  deadTime(5000.0),
	  minExtrusionSpeed(0.55),
	  minExtrusionLength(5.0),
	  minBaseValue(0.15),
	  maxBaseValue(0.45),
	  extrusionCommandedThisSegment(0.0),
	  failCounter(0),
	  lastBaseValueTime(0),
	  lastActiveTime(0),
	  extrusionStartingTime(0),
	  lastSegmentTime(0),
	  steadySinceTime(0),
	  steadyCounter(0),
	  extruder(LogicalDriveToExtruder(drv))
{
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

FilamentSensorStatus SimpleFilamentMonitor::CheckFilament(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept
{
	const uint32_t now = millis();
	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	currentValue = reprap.GetHeat().GetAveragePWM((size_t)heaterNumber);

	bool useFanOffset = false;

	const Tool * const ct = reprap.GetCurrentTool();
	if (ct != nullptr)
	{
		FansBitmap fanMapping = ct->GetFanMapping();
		for (size_t fi = 0; fi < MaxFans; ++fi)
		{
			if (fanMapping.IsBitSet(fi))
			{
				if( reprap.GetFansManager().GetFanValue(fi) > 0)
				{
					useFanOffset = true;
					break;
				}
			}
		}
	}

	if(filamentConsumed > 0.0) {
		uint32_t lastSegmentDelay = (now - lastSegmentTime);
		float mmPerSecSegment = 0.0;

		if( extrusionStartingTime == 0)
		{
			extrusionStartingTime = lastSegmentTime;
			extrusionEndTime = now;
		}

		if( lastSegmentDelay > 0 )
		{
			mmPerSecSegment = (float)((fabs(filamentConsumed) * (double)1000.0) / (double)lastSegmentDelay);
		}
		else
		{
			mmPerSecSegment = 0;
		}

		// check if it is really extruding and if it is not to fast
		// slower than 600 mm/min
		if( mmPerSecSegment >= 0 && mmPerSecSegment < 10.0)
		{
			extrusionLastSegment += filamentConsumed;
			mmPerSec = (float)((fabs(extrusionLastSegment) * (double)1000.0) / (double)(now - extrusionStartingTime));
			extrusionEndTime = now;
		}
	}

	if( filamentConsumed == 0 )
	{
		if(extrusionStartingTime != 0 && (now - extrusionEndTime) > 200) {
			extrusionStartingTime = 0;
			extrusionCommandedThisSegment += extrusionLastSegment;
			extrusionLastSegment = 0.0;
			mmPerSec = 0.0;
		}
	}

	if(currentValue > 0.05 && currentValue < 0.95)
	{
		if( (now - lastActiveTime) > 500 )
		{
			lastActiveTime = now;

			if ((float)fabs(lastValue - currentValue) < 0.01 )
			{
				if(steadySinceTime == 0)
				{
					steadySinceTime = now;
					steadyCounter = 0;
					lastValue = currentValue;
				} else {
					if( steadyCounter < 10 )
					{
						steadyCounter++;
					}
					// if last extrison is longer than 30s ago, reset steady state
					if(extrusionStartingTime == 0 && (now - extrusionEndTime) > 30000) {
						steadySinceTime = now;
						steadyCounter = 0;
					}
				}
			}
			// if it is huge / reset steady state
			else if ((float)fabs(lastValue - currentValue) > 0.20 )
			{
				steadySinceTime = 0;
				steadyCounter = 0;
				lastValue = currentValue;
			}
			else
			{
				if( steadyCounter > 0 )
				{
					steadyCounter--;
				}
				else
				{
					steadySinceTime = 0;
					lastValue = currentValue;
				}
			}
		}
	}
	else
	{
		steadySinceTime = 0;
	}

	if(steadySinceTime != 0 && (now - steadySinceTime) > deadTime )
	{
		if((now - lastBaseValueTime) > 500)
		{
			lastBaseValueTime = now;

			/*if( mmPerSec > 0 && mmPerSec < minExtrusionSpeed)
			{
				// compensate the slow extrusion
				baseValue = fmax(minBaseValue, fmin(maxBaseValue, ((1.0 - 0.1 ) * baseValue) + (currentValue - slope * mmPerSec - ( useFanOffset?fanOffset:0.0)) * 0.1));
			}
			else if(mmPerSec == 0)
			{
				baseValue = fmax(minBaseValue, fmin(maxBaseValue, ((1.0 - 0.1 ) * baseValue) + (currentValue - ( useFanOffset?fanOffset:0.0)) * 0.1));
			}*/

			expectedValue = ((baseValue) + slope * mmPerSec) * 0.9 + ( useFanOffset?fanOffset:0.0);

			if( extrusionLastSegment > minExtrusionLength )
			{
				if( currentValue < expectedValue)
				{
					if(failCounter < 10)
						failCounter++;
				}
				else
				{
					if(failCounter > 0)
						failCounter--;
				}
			}
		}
	}
	else
	{
		failCounter = 0;
	}

	if( failCounter > 5)
	{
		ret = FilamentSensorStatus::tooLittleMovement;
	}
	/*if( steadySinceTime != 0 && (now - steadySinceTime) > 30000 )
	{
		if( isPrinting && (maxBaseValue - minBaseValue) < 0.1 )
		{
			ret = FilamentSensorStatus::tooLittleMovement;
		}
		steadySinceTime = 0;
	}*/

	// check if the time belongs to current segment
	/*if(lastSegmentDelay > 200) {
		lastSegmentTime = 0;
	}

	// track the start of extruding
	if( filamentConsumed != 0 )
	{
		if( extrusionStartingTime == 0 )
		{
			extrusionStartingTime = lastSegmentTime!=0?lastSegmentTime:now;
		}
		if(steadySinceTime != 0)
		{
			filamentConsumedSinceSteady = true;
		}
	}
	else
	{
		if(steadySinceTime != 0 && (now - steadySinceTime) > 30000 )
		{
			steadySinceTime = 0;
		}
	}

	// no time is available at the beginning of at extrusion move
	if( lastSegmentTime !=0 && extrusionLastSegment != 0) {
		mmPerSec = ((float)fabs(extrusionLastSegment)) / (now - lastSegmentTime) / 1000.0;
	}
	else
	{
		mmPerSec = 0;
	}

	// check if it is to fast for extrusion
	// possible retraction move
	if( extrusionLastSegment > 0 && mmPerSec > 0 && mmPerSec < 10.0 )
	{
		extrusionCommandedThisSegment += extrusionLastSegment;
		extrusionLastSegment = 0;
		mmPerSec = ((float)fabs(extrusionCommandedThisSegment)) / (now - extrusionStartingTime) / 1000.0;
		extrusionStartingTime = 0;
	}
	else
	{
		mmPerSec = 0;
		extrusionLastSegment = 0.0;
		extrusionStartingTime = 0.0;
	}*/




	/*if( extrusionCommandedThisSegment > 0 && extrusionLastSegment == 0 )
	{
		if(currentValue > 0.05 && currentValue < 0.95)
		{
			if ((float)fabs(lastValue - currentValue) < 0.05)
			{
				if(steadySinceTime == 0)
				{
					steadySinceTime = now;
				}
			}
			else
			{
				steadySinceTime = 0;
			}
		}
		else
		{
			steadySinceTime = 0;
		}

		if( isPrinting && steadySinceTime != 0 && (now - steadySinceTime) > 30000 )
		{
			ret = FilamentSensorStatus::tooLittleMovement;
			steadySinceTime = 0;
		}*/

		/*const float delay = (now - extrusionStartingTime);
		if( delay > 0 )
		{
			mmPerSec = (extrusionCommandedThisSegment * 1000.0) / delay;
		}

		bool useFanOffset = false;

		const Tool * const ct = reprap.GetCurrentTool();
		if (ct != nullptr)
		{
			FansBitmap fanMapping = ct->GetFanMapping();
			for (size_t fi = 0; fi < MaxFans; ++fi)
			{
				if (fanMapping.IsBitSet(fi))
				{
					if( reprap.GetFansManager().GetFanValue(fi) > 0)
					{
						useFanOffset = true;
						break;
					}
				}
			}
		}

		expectedValue = ((baseValue + ( useFanOffset?fanOffset:0.0)) + slope * mmPerSec) * 0.9;

		// is more than minExtrusionLength filament extruded?
		// or at begin of extruding wait for dead time
		if (extrusionCommandedThisSegment > minExtrusionLength && ( (now - extrusionStartingTime) > deadTime ))
		{
			if( mmPerSec > minExtrusionSpeed)
			{
				if( currentValue < expectedValue )
				{
					if(failCounter < 2)
					{
						failCounter++;
					}
					else
					{
						ret = FilamentSensorStatus::tooLittleMovement;
					}
				}
				else
				{
					if( failCounter != 0 )
					{
						failCounter--;
					}
				}
			}

			// discard the values
			extrusionCommandedThisSegment = 0.0;
			extrusionStartingTime = 0;
		}

		// if we are driving very slow, we can recalculate the base value
		if( filamentConsumed > 0 && mmPerSec > minExtrusionSpeed )
		{
			lastActiveTime = now;
		}

		if( (now - lastActiveTime) > 500 )
		{
			// wait 100 millis for the next step
			lastActiveTime = now - 400;
			// built the average of the average pwm, but wait for stable values
			if (currentValue > 0.05 && (float)fabs(lastValue - currentValue) < 0.05)
			{
				if( mmPerSec > 0 && mmPerSec < minExtrusionSpeed)
				{
					// compensate the slow extrusion
					baseValue = fmax(minBaseValue, fmin(maxBaseValue, ((1.0 - 0.3 ) * baseValue) + (currentValue - slope * mmPerSec - ( useFanOffset?fanOffset:0.0)) * 0.3));
				}
				else
				{
					baseValue = fmax(minBaseValue, fmin(maxBaseValue, ((1.0 - 0.3 ) * baseValue) + (currentValue - ( useFanOffset?fanOffset:0.0)) * 0.3));
				}
			}

			extrusionCommandedThisSegment = 0.0;
			extrusionStartingTime = 0;
			expectedValue = 0.0;
			mmPerSec = 0.0;
			failCounter = 0;
		}*/
	//}

	if (filamentPresent == false)
	{
		ret = FilamentSensorStatus::noFilament;
	}

	/*if( extrusionStartingTime == 0 )
	{
		extrusionCommandedThisSegment = 0.0;
		mmPerSec = 0.0;
	}*/

	lastSegmentTime = now;

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
