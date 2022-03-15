/*
 * AdvancedFilamentSensor.cpp
 * based on SimpleFilamentSensor.cpp
 * from David
 *
 *  Created on: 01 February 2022
 *      Author: Tim Schneider
 *
 *  The advanced filament monitor is using a simple switch as a filament detector and is watching the energy consumption of the
 *  hotend in use. The assumption is as follows:
 *  	1) the energy consumption will rise with rising filament extrusion per second
 *  	2) the energy consumption will settle while not extruding
 *  	3) the energy consumption will decrease or stay near the same level as 2) while extruding with no or grinded filament
 *
 *  In order to detect no or grinded filament, the advanced filament monitor is:
 *  	x) calibrating the idle energy consumption at printing temperature
 *  	2) checking the current current energy consumption while extruding and if the consumption is below the threshold, the
 *  	   print will be paused.
 *
 * feedrate in mm/sec
 * heater power in Watts P = U² / R * averagePWM
 * U in Volts defaults to reprap.GetPlatform().GetCurrentPowerVoltage()
 * R in Ohms is set by parameter
 * averagePWM is the turned on percentage % reprap.GetHeat().GetAveragePWM(heater)
 * limit in W
 */

#include "AdvancedFilamentMonitor.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(AdvancedFilamentMonitor, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(AdvancedFilamentMonitor, __VA_ARGS__)

constexpr ObjectModelTableEntry AdvancedFilamentMonitor::objectModelTable[] =
{
	// 0. Within each group, these entries must be in alphabetical order
	{ "calibrated", 	OBJECT_MODEL_FUNC_IF(self->HaveCalibrationData(), self, 1), 	ObjectModelEntryFlags::live },
	{ "enabled",		OBJECT_MODEL_FUNC(self->comparisonEnabled),						ObjectModelEntryFlags::none },
	{ "status",			OBJECT_MODEL_FUNC(self->GetStatusText()),						ObjectModelEntryFlags::live },
	{ "type",			OBJECT_MODEL_FUNC_NOSELF("advanced"), 							ObjectModelEntryFlags::none },

	// 1. AdvancedFilamentMonitor.calibrated members
	{ "mmPerWatt",		OBJECT_MODEL_FUNC(self->MeasuredSensitivity(), 3), 														ObjectModelEntryFlags::live },
	{ "percentMin",		OBJECT_MODEL_FUNC(ConvertToPercent(self->minMovementRatio)), 											ObjectModelEntryFlags::live },
	{ "totalDistance",	OBJECT_MODEL_FUNC(self->totalExtrusionCommanded, 1), 													ObjectModelEntryFlags::live },

	// 2. AdvancedFilamentMonitor.configured members
	{ "mmPerWatt",		OBJECT_MODEL_FUNC(self->mmPerWatt, 3), 																ObjectModelEntryFlags::none },
	{ "percentMin",		OBJECT_MODEL_FUNC(ConvertToPercent(self->minMovementAllowed)), 											ObjectModelEntryFlags::none },
	{ "sampleDistance", OBJECT_MODEL_FUNC(self->minimumExtrusionCheckLength, 1), 												ObjectModelEntryFlags::none },
};

constexpr uint8_t AdvancedFilamentMonitor::objectModelTableDescriptor[] = { 3, 4, 3, 3 };

DEFINE_GET_OBJECT_MODEL_TABLE(AdvancedFilamentMonitor)

#endif

AdvancedFilamentMonitor::AdvancedFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept
	: FilamentMonitor(drv, monitorType, did),
	  mmPerWatt(DefaultMmPerWatt),
	  minMovementAllowed(DefaultMinMovementAllowed),
	  minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength),
	  comparisonEnabled(false),
	  highWhenNoFilament(monitorType == 2),
	  filamentPresent(false),
	  enabled(false)
{
}

void AdvancedFilamentMonitor::Init() noexcept
{
	sensorValue = 0;
	calibrationStarted = false;
	Reset();
}

void AdvancedFilamentMonitor::Reset() noexcept
{
	extrusionCommandedThisSegment = movementMeasuredThisSegment = 0.0;
	comparisonStarted = false;
}

bool AdvancedFilamentMonitor::HaveCalibrationData() const noexcept
{
	return calibrationStarted && fabsf(totalMovementMeasured) > 1.0 && totalExtrusionCommanded > 20.0;
}

float AdvancedFilamentMonitor::MeasuredSensitivity() const noexcept
{
	return totalExtrusionCommanded/totalMovementMeasured;
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult AdvancedFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException)
{
	const GCodeResult rslt = CommonConfigure(gb, reply, InterruptMode::none, seen);
	if (Succeeded(rslt))
	{
		gb.TryGetFValue('L', mmPerWatt, seen);
		gb.TryGetFValue('E', minimumExtrusionCheckLength, seen);

		if (gb.Seen('R'))
		{
			seen = true;
			minMovementAllowed = (float)gb.GetUIValue() * 0.01;
		}

		if (gb.Seen('S'))
		{
			seen = true;
			comparisonEnabled = (gb.GetIValue() > 0);
		}

		if (seen)
		{
			Init();
			reprap.SensorsUpdated();
		}
		else
		{
			reply.copy("Advanced filament sensor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, output %s when no filament, filament present: %s",
						(comparisonEnabled) ? "enabled" : "disabled",
						(highWhenNoFilament) ? "high" : "low",
						(filamentPresent) ? "yes" : "no");
		}
	}
	return rslt;
}

// ISR for when the pin state changes
bool AdvancedFilamentMonitor::Interrupt() noexcept
{
	// Nothing needed here
	GetPort().DetachInterrupt();
	return false;
}

// Call the following regularly to keep the status up to date
void AdvancedFilamentMonitor::Poll() noexcept
{
	const bool b = GetPort().ReadDigital();
	filamentPresent = (highWhenNoFilament) ? !b : b;
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
FilamentSensorStatus AdvancedFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept
{
	Poll();
	if(isPrinting == true) {

	}
	return (!comparisonEnabled || filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus AdvancedFilamentMonitor::Clear() noexcept
{
	Poll();
	return (!comparisonEnabled || filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Print diagnostic info for this sensor
void AdvancedFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder) noexcept
{
	Poll();
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: %s\n", extruder, (filamentPresent) ? "ok" : "no filament");
}

#if SUPPORT_REMOTE_COMMANDS

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult AdvancedFilamentMonitor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	const GCodeResult rslt = CommonConfigure(parser, reply, InterruptMode::none, seen);
	if (rslt <= GCodeResult::warning)
	{
		uint16_t temp;
		if (parser.GetUintParam('S', temp))
		{
			seen = true;
			comparisonEnabled = (temp > 0);
		}


		if (seen)
		{
			Check(false, false, 0, 0.0);
		}
		else
		{
			reply.copy("Advanced filament sensor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, output %s when no filament, filament present: %s",
						(comparisonEnabled) ? "enabled" : "disabled",
						(highWhenNoFilament) ? "high" : "low",
						(filamentPresent) ? "yes" : "no");
		}
	}
	return rslt;
}

#endif

// End
