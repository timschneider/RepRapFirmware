/*
 * SimpleFilamentSensor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_SIMPLEFILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_SIMPLEFILAMENTMONITOR_H_

#include "FilamentMonitor.h"

class SimpleFilamentMonitor : public FilamentMonitor
{
public:
	SimpleFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException) override;
#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;
#endif
	FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept override;
	FilamentSensorStatus CheckFilament(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept;
	FilamentSensorStatus Clear() noexcept override;
	void Diagnostics(MessageType mtype, unsigned int extruder) noexcept override;
	bool Interrupt() noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:
	void Poll() noexcept;
	int GetToolNumberForDrive() noexcept;

	bool highWhenNoFilament;
	bool filamentPresent;
	bool enabled;
	float baseValue;
	float currentValue;
	float lastValue;
	float expectedValue;
	float mmPerSec;
	float fanOffset;
	float slope;
	float deadTime;
	float minExtrusionSpeed;
	float minExtrusionLength;
	float minBaseValue;
	float maxBaseValue;
	float extrusionLastSegment;
	float extrusionCommandedThisSegment;					// the amount of extrusion commanded (mm) since we last did a comparison
	int heaterNumber;
	unsigned int failCounter;

	uint32_t lastBaseValueTime;
	uint32_t lastActiveTime;
	uint32_t extrusionStartingTime;
	uint32_t extrusionEndTime;
	uint32_t lastSegmentTime;
	uint32_t steadySinceTime;
	int steadyCounter;

	unsigned int extruder;
};

#endif /* SRC_FILAMENTSENSORS_SIMPLEFILAMENTMONITOR_H_ */
