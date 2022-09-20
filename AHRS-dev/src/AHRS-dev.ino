/*
 Name:		AHRS_dev.ino
 Created:	9/17/22 23:05:32
 Author:	alexh
*/

// the setup function runs once when you press reset or power the board
#include "Fusion.h"
#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

SFE_MMC5983MA mag;
SparkFun_ISM330DHCX ism;

sfe_ism_data_t gyroData;
sfe_ism_data_t accelData;

FusionVector gyroscope;
FusionVector accelerometer;
FusionVector magnetometer;

FusionAhrs ahrs;

FusionEuler attitude;

float gyroOffset[3] = { 0.0024f, -0.0104f, -0.0043f };
float accelOffset[3] = { -57.73f / 1000.0f, -18.71f / 1000.0f, 7.735f / 1000.0f };
float magHardOffset[3] = { -55.55f, 80.53f, 145.28f };
float magSoftOffset[3][3] = { { 0.945f, 0.008f, 0.011f }, { 0.008f, 1.131f, 0.038f }, { 0.011f, 0.038f, 0.936f } };

unsigned long t_last = 0;
float t_delta = 0;

void getMag(FusionVector *magnetometer) {
#define M magnetometer -> axis
	M.x = ((float)mag.getMeasurementX() - 131072.0) * 0.0061035 - magHardOffset[0];
	M.y = ((float)mag.getMeasurementY() - 131072.0) * 0.0061035 - magHardOffset[1];
	M.z = ((float)mag.getMeasurementZ() - 131072.0) * 0.0061035 - magHardOffset[2];
	M.x = M.x * magSoftOffset[0][0] + M.x * magSoftOffset[0][1] + M.x * magSoftOffset[0][2];
	M.y = M.y * magSoftOffset[1][0] + M.y * magSoftOffset[1][1] + M.y * magSoftOffset[1][2];
	M.z = -M.z * magSoftOffset[2][0] - M.z * magSoftOffset[2][1] - M.z * magSoftOffset[2][2];
#undef M
}

void offsetGyro() {
	gyroData.xData -= gyroOffset[0];
	gyroData.yData -= gyroOffset[1];
	gyroData.zData -= gyroOffset[2];
}

void offsetAccel() {
	accelData.xData -= accelOffset[0];
	accelData.yData -= accelOffset[1];
	accelData.zData -= accelOffset[2];
}

FusionVector sfeToVector(sfe_ism_data_t data) {
	FusionVector vector;
	vector.axis.x = data.xData / 1000.0f;
	vector.axis.y = data.yData / 1000.0f;
	vector.axis.z = data.zData / 1000.0f;
	return vector;
}

void setup() {
	Serial.begin(115200);

	Wire.begin();

	if (!ism.begin())//check if ism begins
	{
		Serial.println(F("ism did not begin"));
		while (1);
	}

	if (!mag.begin())//check if mag begins
	{
		Serial.println(F("mag did not begin"));
		while (1);
	}

	//reset ism
	ism.deviceReset();
	while (!ism.getDeviceReset())
	{
		delay(1);
	}
	Serial.println(F("ism reset"));

	//reset mag
	mag.softReset();
	Serial.println(F("mag reset"));

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
	Serial.println(F("Starting..."));

	FusionAhrsInitialise(&ahrs);
}

// the loop function runs over and over again until power down or reset
void loop() {
	t_last = millis();

	ism.getAccel(&accelData);
	ism.getGyro(&gyroData);
	getMag(&magnetometer);

	offsetAccel();
	offsetGyro();

	gyroscope = sfeToVector(gyroData);
	accelerometer = sfeToVector(accelData);

	FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, t_delta);

	attitude = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

	Serial.print(attitude.angle.pitch); Serial.print('\t');
	Serial.print(attitude.angle.yaw); Serial.print('\t');
	Serial.print(attitude.angle.roll); Serial.print('\t');

	t_delta = (millis() - t_last) / 1000.0f;
	Serial.println(t_delta);
}