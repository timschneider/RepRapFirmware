/*
 * StallDetectionEndstop.h
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_
#define SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_

#include "Endstop.h"

// Motor stall detection endstop
class StallDetectionEndstop final : public Endstop
{
public:
	void* operator new(size_t sz) { return FreelistManager::Allocate<StallDetectionEndstop>(); }
	void operator delete(void* p) { FreelistManager::Release<StallDetectionEndstop>(p); }

	StallDetectionEndstop(uint8_t axis, EndStopPosition pos, bool p_individualMotors);		// for creating axis endstops
	StallDetectionEndstop();							// for creating the single extruders endstop

	EndStopType GetEndstopType() const override { return (individualMotors) ? EndStopType::motorStallIndividual : EndStopType::motorStallAny; }
	EndStopHit Stopped() const override;
	bool Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) override;
	EndstopHitDetails CheckTriggered(bool goingSlow) override;
	bool Acknowledge(EndstopHitDetails what) override;
	void AppendDetails(const StringRef& str) override;
	void SetDrivers(DriversBitmap extruderDrivers);		// for setting which local extruder drives are active extruder endstops

private:
	DriversBitmap driversMonitored;
	unsigned int numDriversLeft;
	bool individualMotors;
	bool stopAll;
};

#endif /* SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_ */
