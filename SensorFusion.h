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
		gpsAltTimestamps[gpsAltSampleNext] = gpsTimestamp;
		gpsAltSampleNext = (gpsAltSampleNext + 1) % GPS_ALT_SAMPLES;
		if (gpsAltSampleCount < GPS_ALT_SAMPLES)
			gpsAltSampleCount++;

		anchorGpsAltMeters = fitGpsAltitudeAt(gpsTimestamp);
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
	float fitGpsAltitudeAt(uint32_t gpsTimestamp) const {
		if (gpsAltSampleCount == 1)
			return gpsAltSamples[0];

		double sumT = 0;
		double sumAlt = 0;
		double sumTT = 0;
		double sumTAlt = 0;

		for (int i = 0; i < gpsAltSampleCount; i++) {
			double t = ((int32_t)(gpsAltTimestamps[i] - gpsTimestamp)) / 1000.0;
			double alt = gpsAltSamples[i];
			sumT += t;
			sumAlt += alt;
			sumTT += t * t;
			sumTAlt += t * alt;
		}

		double n = gpsAltSampleCount;
		double denom = n * sumTT - sumT * sumT;
		if (denom == 0)
			return gpsAltSamples[(gpsAltSampleNext + GPS_ALT_SAMPLES - 1) % GPS_ALT_SAMPLES];

		double slope = (n * sumTAlt - sumT * sumAlt) / denom;
		double intercept = (sumAlt - slope * sumT) / n;
		return intercept;
	}

	float gpsAltSamples[GPS_ALT_SAMPLES];
	uint32_t gpsAltTimestamps[GPS_ALT_SAMPLES];
	int gpsAltSampleCount = 0;
	int gpsAltSampleNext = 0;
	uint32_t lastGpsAltTimestamp = 0;
	float anchorGpsAltMeters = 0;
	float anchorPressureAltFeet = 0;
	bool hasAltitudeAnchor = false;
	bool hasPressureAnchor = false;
};
