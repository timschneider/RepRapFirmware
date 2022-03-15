/*
 * AdvancedFilamentMonitor.h
 *
 *  Created on: 01.02.2022
 *      Author: Tim Schneider
 */

#ifndef SRC_FILAMENTMONITORS_ADVANCEDFILAMENTMONITOR_H_
#define SRC_FILAMENTMONITORS_ADVANCEDFILAMENTMONITOR_H_


#include "FilamentMonitor.h"

class AdvancedFilamentMonitor : public FilamentMonitor
{
public:
	AdvancedFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException) override;
#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;
#endif
	FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept override;
	FilamentSensorStatus Clear() noexcept override;
	void Diagnostics(MessageType mtype, unsigned int extruder) noexcept override;
	bool Interrupt() noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:
	static constexpr float DefaultMmPerWatt = 1.0;
	static constexpr float DefaultMinMovementAllowed = 0.6;
	static constexpr float DefaultMinimumExtrusionCheckLength = 30.0;

	void Init() noexcept;
	void Reset() noexcept;
	void Poll() noexcept;

	bool HaveCalibrationData() const noexcept;
	float MeasuredSensitivity() const noexcept;

	// Configuration parameters
	float mmPerWatt;
	float minMovementAllowed;
	float minimumExtrusionCheckLength;
	bool comparisonEnabled;

	// Other data
	uint32_t sensorValue;									// how many watts counted
	uint32_t lastIsrTime;									// the time we recorded an interrupt

	float extrusionCommandedThisSegment;					// the amount of extrusion commanded (mm) since we last did a comparison
	float movementMeasuredThisSegment;						// the accumulated movement in complete rotations since the previous comparison

	// Values measured for calibration
	float minMovementRatio;
	float totalExtrusionCommanded;
	float totalMovementMeasured;

	bool comparisonStarted;
	bool calibrationStarted;

	bool highWhenNoFilament;
	bool filamentPresent;
	bool enabled;
};


#endif /* SRC_FILAMENTMONITORS_ADVANCEDFILAMENTMONITOR_H_ */
