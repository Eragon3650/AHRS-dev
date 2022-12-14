/**
 * @file FusionAhrs.h
 * @author Seb Madgwick
 * @brief AHRS algorithm to combine gyroscope, accelerometer, and magnetometer
 * measurements into a single measurement of orientation relative to the Earth.
 */

#ifndef FUSION_AHRS_H
#define FUSION_AHRS_H

 //------------------------------------------------------------------------------
 // Includes

#include "FusionMath.h"
#include <stdbool.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief AHRS algorithm settings.
 */
typedef struct {
	float gain;
	float accelerationRejection;
	float magneticRejection;
	unsigned int rejectionTimeout;
} FusionAhrsSettings;

/**
 * @brief AHRS algorithm structure.  Structure members are used internally and
 * must not be accessed by the application.
 */
typedef struct {
	FusionAhrsSettings settings;
	FusionQuaternion quaternion;
	FusionVector accelerometer;
	bool initialising;
	float rampedGain;
	float rampedGainStep;
	FusionVector halfAccelerometerFeedback;
	FusionVector halfMagnetometerFeedback;
	bool accelerometerIgnored;
	unsigned int accelerationRejectionTimer;
	bool accelerationRejectionTimeout;
	bool magnetometerIgnored;
	unsigned int magneticRejectionTimer;
	bool magneticRejectionTimeout;
} FusionAhrs;

/**
 * @brief AHRS algorithm internal states.
 */
typedef struct {
	float accelerationError;
	bool accelerometerIgnored;
	float accelerationRejectionTimer;
	float magneticError;
	bool magnetometerIgnored;
	float magneticRejectionTimer;
} FusionAhrsInternalStates;

/**
 * @brief AHRS algorithm flags.
 */
typedef struct {
	bool initialising;
	bool accelerationRejectionWarning;
	bool accelerationRejectionTimeout;
	bool magneticRejectionWarning;
	bool magneticRejectionTimeout;
} FusionAhrsFlags;

//------------------------------------------------------------------------------
// Function declarations

void FusionAhrsInitialise(FusionAhrs* const ahrs);

void FusionAhrsReset(FusionAhrs* const ahrs);

void FusionAhrsSetSettings(FusionAhrs* const ahrs, const FusionAhrsSettings* const settings);

void FusionAhrsUpdate(FusionAhrs* const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const FusionVector magnetometer, const double deltaTime);

void FusionAhrsUpdateNoMagnetometer(FusionAhrs* const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const double deltaTime);

void FusionAhrsUpdateExternalHeading(FusionAhrs* const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float heading, const float deltaTime);

FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs* const ahrs);

FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs* const ahrs);

FusionVector FusionAhrsGetEarthAcceleration(const FusionAhrs* const ahrs);

FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs* const ahrs);

FusionAhrsFlags FusionAhrsGetFlags(FusionAhrs* const ahrs);

void FusionAhrsSetHeading(FusionAhrs* const ahrs, const float heading);

#endif

//------------------------------------------------------------------------------
// End of file
