#pragma once

#include <math.h>
#include <stdint.h>
#include "dataTools.h"

class SensorFusion {
public:
	static const int GPS_ALT_SAMPLES = 5;
	static constexpr double EARTH_MEAN_RADIUS_METERS = 6371000.0;

	struct Position {
		double lat = 0;
		double lon = 0;
	};

	void updateGpsPosition(double lat, double lon, float track, float hvelKnots, uint32_t gpsTimestamp) {
		gpsLat = lat;
		gpsLon = lon;
		gpsTrack = track;
		gpsHvelKnots = hvelKnots;
		lastGpsPositionTimestamp = gpsTimestamp;
		hasGpsPosition = true;
	}

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

	Position fusedPosition(uint32_t nowMs) const {
		if (!hasGpsPosition)
			return Position();

		uint32_t fixAgeMs = nowMs - lastGpsPositionTimestamp;
		double distanceMeters = gpsHvelKnots * 0.514444 * fixAgeMs / 1000.0;
		return locationBearingDistance(gpsLat, gpsLon, gpsTrack, distanceMeters);
	}

private:
	Position locationBearingDistance(double lat, double lon, double brng, double distanceMeters) const {
		double lat1 = lat * M_PI / 180;
		double lon1 = lon * M_PI / 180;
		double brngRad = brng * M_PI / 180;
		double angularDistance = distanceMeters / EARTH_MEAN_RADIUS_METERS;

		double lat2 = asin(sin(lat1) * cos(angularDistance) +
			cos(lat1) * sin(angularDistance) * cos(brngRad));
		double lon2 = lon1 + atan2(
			sin(brngRad) * sin(angularDistance) * cos(lat1),
			cos(angularDistance) - sin(lat1) * sin(lat2));

		Position pos;
		pos.lat = lat2 * 180 / M_PI;
		pos.lon = lon2 * 180 / M_PI;
		return pos;
	}

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

	double gpsLat = 0;
	double gpsLon = 0;
	float gpsTrack = 0;
	float gpsHvelKnots = 0;
	uint32_t lastGpsPositionTimestamp = 0;
	bool hasGpsPosition = false;
};
