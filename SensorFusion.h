#pragma once

#include <stdint.h>
#include "dataTools.h"

class SensorFusion {
public:
	static const int GPS_ALT_SAMPLES = 5;

	void updateGpsAltitude(float altMeters, uint32_t gpsTimestamp, float pressureAltFeet, bool hasPressureAlt) {
		if (gpsTimestamp == lastGpsAltTimestamp)
			return;
		lastGpsAltTimestamp = gpsTimestamp;

		gpsAltSamples[gpsAltSampleNext] = altMeters;
		gpsAltSampleNext = (gpsAltSampleNext + 1) % GPS_ALT_SAMPLES;
		if (gpsAltSampleCount < GPS_ALT_SAMPLES)
			gpsAltSampleCount++;

		float sum = 0;
		for (int i = 0; i < gpsAltSampleCount; i++)
			sum += gpsAltSamples[i];
		anchorGpsAltMeters = sum / gpsAltSampleCount;
		anchorPressureAltFeet = pressureAltFeet;
		hasPressureAnchor = hasPressureAlt;
		hasAltitudeAnchor = true;
	}

	bool hasFusedAltitude() const {
		return hasAltitudeAnchor;
	}

	float fusedAltitudeMeters(float fallbackGpsAltMeters, float pressureAltFeet, bool hasPressureAlt) const {
		if (!hasAltitudeAnchor)
			return fallbackGpsAltMeters;
		if (!hasPressureAnchor || !hasPressureAlt)
			return anchorGpsAltMeters;

		// GPS is the absolute reference. Pressure altitude is used only for
		// high-rate delta since the last GPS altitude anchor.
		float pressureDeltaMeters = (pressureAltFeet - anchorPressureAltFeet) / FEET_PER_METER;
		return anchorGpsAltMeters + pressureDeltaMeters;
	}

private:
	float gpsAltSamples[GPS_ALT_SAMPLES];
	int gpsAltSampleCount = 0;
	int gpsAltSampleNext = 0;
	uint32_t lastGpsAltTimestamp = 0;
	float anchorGpsAltMeters = 0;
	float anchorPressureAltFeet = 0;
	bool hasAltitudeAnchor = false;
	bool hasPressureAnchor = false;
};
