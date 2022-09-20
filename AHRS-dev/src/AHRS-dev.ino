/*
 Name:		AHRS_dev.ino
 Created:	9/17/22 23:05:32
 Author:	alexh
*/

// the setup function runs once when you press reset or power the board
#include "AHRSMath.h"
#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

SFE_MMC5983MA mag;
SparkFun_ISM330DHCX ism;

sfe_ism_data_t gyroData;
sfe_ism_data_t accelData;

Vector gyroscope;
Vector accelerometer;
Vector magnetometer;

float gyroOffset[3] = { 0.1521, 0.0098, 0.0043 };
float accelOffset[3] = { -57.73 * 1E-3, -18.71 * 1E-3, 7.735 * 1E-3 };
float magHardOffset[3] = { -60.15, -39.55, 28.15 };
float magSoftOffset[3][3] = { { 0.924, -0.017, -0.001 }, { -0.017, 1.166, 0.023 }, { -0.001, 0.023, 0.929 } };

unsigned long t_last = 0;
float t_delta = 0;

Quaternion attitude = { 0.0, 0.0, 0.0, 1.0 };

#define INIT_GAIN 10.0f
#define INIT_PERIOD 3.0f

const AhrsSettings settings = { 0.4f, 90.0f, 90.0f, 0 };
const float rampedGainStep = (INIT_GAIN - settings.gain) / INIT_PERIOD;
float rampedGain = INIT_GAIN;
bool initializing = true;
bool accelIgnored = false;
unsigned int accelRejectTimer = 0;
bool accelRejectTimeout = false;
bool magIgnored = false;
unsigned int magRejectTimer = 0;
bool magRejectTimeout = false;

Quaternion madgwick(Quaternion quaternion, Vector gyro, Vector accel, Vector mag, float deltaTime) {
#define last quaternion.element
	Quaternion result;
	if (initializing) {
		rampedGain -= rampedGainStep * deltaTime;
		if (rampedGain < settings.gain) {
			rampedGain = settings.gain;
			initializing = false;
			accelRejectTimeout = false;
		}
	}
	Vector halfGravity = {
		last.x * last.z - last.w * last.y,
		last.y * last.z + last.w * last.x,
		last.w * last.w - 0.5f + last.z * last.z
	};
	Vector halfWest = {
		last.x * last.y + last.w * last.z,
		last.w * last.w - 0.5f + last.y * last.y,
		last.y * last.z - last.w * last.x
	};

	Vector halfAccelFeedback = VECTOR_ZERO;
	if (!VectorIsZero(accel)) {
		halfAccelFeedback = VectorCrossProduct(VectorNormalize(accel), halfGravity);

		if ((initializing) || (VectorMagnitudeSquared(halfAccelFeedback) <= settings.accelRejection)) {
			accelIgnored = false;
			accelRejectTimer -= accelRejectTimer >= 10 ? 10 : 0;
		}
		else {
			accelRejectTimer++;
		}
	}

	Vector halfMagFeedback = VECTOR_ZERO;
	magIgnored = true;
	if (!VectorIsZero(mag)) {
		magRejectTimeout = false;
		Vector halfMagFeedback = VectorCrossProduct(VectorNormalize(VectorCrossProduct(halfGravity, mag)), halfWest);
		if ((initializing) || (VectorMagnitudeSquared(mag) <= settings.magRejection)) {
			magIgnored = false;
			magRejectTimer -= magRejectTimer >= 10 ? 10 : 0;
		}
		else {
			magRejectTimer++;
		}
	}
	const Vector halfGyro = VectorMultiplyScalar(gyro, TO_RAD * 0.5f);
	const Vector adjustedHalfGyro = VectorAdd(halfGyro, VectorMultiplyScalar(VectorAdd(halfAccelFeedback, halfMagFeedback), rampedGain));
	result = QuaternionNormalize(QuaternionAdd(quaternion, QuaternionMultiplyVector(quaternion, VectorMultiplyScalar(adjustedHalfGyro, deltaTime))));
	return result;
#undef last
}

inline float ieee_float(uint32_t f) {
	static_assert(sizeof(float) == sizeof f, "`float` has a weird size.");
	float ret;
	memcpy(&ret, &f, sizeof(float));
	return ret;
}

Vector getMag() {
#define M magnetometer.axis
	M.x = ((ieee_float(mag.getMeasurementX()) - 131072) / 131072) * 800 - magHardOffset[0];
	M.y = ((ieee_float(mag.getMeasurementY()) - 131072) / 131072) * 800 - magHardOffset[1];
	M.z = ((ieee_float(mag.getMeasurementZ()) - 131072) / 131072) * 800 - magHardOffset[2];
	M.x = M.x * magSoftOffset[0][0] + M.x * magSoftOffset[0][1] + M.x * magSoftOffset[0][2];
	M.y = M.y * magSoftOffset[1][0] + M.y * magSoftOffset[1][1] + M.y * magSoftOffset[1][2];
	M.z = -1.0f * (M.z * magSoftOffset[2][0] + M.z * magSoftOffset[2][1] + M.z * magSoftOffset[2][2]);
	return magnetometer;
#undef M
}

void offsetGyro() {
	gyroData.xData = gyroData.xData /* TO_RAD/**/ - gyroOffset[0];
	gyroData.yData = gyroData.yData /* TO_RAD/**/ - gyroOffset[1];
	gyroData.zData = gyroData.zData /* TO_RAD/**/ - gyroOffset[2];
}

void offsetAccel() {
	accelData.xData -= accelOffset[0];
	accelData.yData -= accelOffset[1];
	accelData.zData -= accelOffset[2];
}

Vector sfeToVector(sfe_ism_data_t data) {
	Vector vector;
	vector.axis.x = data.xData * 1E-3;
	vector.axis.y = data.yData * 1E-3;
	vector.axis.z = data.zData * 1E-3;
	return vector;
}

void setup() {
	Serial.begin(115200);

	Wire.begin();

	if (!ism.begin())//check if ism begins
	{
		Serial.println("ism did not begin");
		while (1);
	}

	if (!mag.begin())//check if mag begins
	{
		Serial.println("mag did not begin");
		while (1);
	}

	//reset ism
	ism.deviceReset();
	while (!ism.getDeviceReset())
	{
		delay(1);
	}
	Serial.println("ism reset");

	//reset mag
	mag.softReset();
	Serial.println("mag reset");

	ism.setDeviceConfig();
	ism.setBlockDataUpdate();

	// Set the output data rate and precision of the accelerometer
	ism.setAccelDataRate(ISM_XL_ODR_104Hz);
	ism.setAccelFullScale(ISM_4g);

	// Set the output data rate and precision of the gyroscope
	ism.setGyroDataRate(ISM_GY_ODR_104Hz);
	ism.setGyroFullScale(ISM_500dps);

	// Turn on the accelerometer's filter and apply settings.
	ism.setAccelFilterLP2();
	ism.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

	// Turn on the gyroscope's filter and apply settings.
	ism.setGyroFilterLP1();
	ism.setGyroLP1Bandwidth(ISM_MEDIUM);
}

// the loop function runs over and over again until power down or reset
void loop() {
	t_last = millis();

	ism.getAccel(&accelData);
	ism.getGyro(&gyroData);

	offsetAccel();
	offsetGyro();

	attitude = madgwick(attitude, sfeToVector(gyroData), sfeToVector(accelData), getMag(), t_delta);

	Euler angle = QuaternionToEuler(attitude);
	Serial.print(angle.angle.pitch); Serial.print('\t');
	Serial.print(angle.angle.roll); Serial.print('\t');
	Serial.print(angle.angle.yaw); Serial.print('\t');

	t_delta = (millis() - t_last) / 1000.0f;
	Serial.println(t_delta);
}