/*
 Name:		AHRS_dev.ino
 Created:	9/17/22 23:05:32
 Author:	alexh
*/

// the setup function runs once when you press reset or power the board
#include "Fusion.h"
#include "MMC5983MA.h"
#include "I2Cdev.h"
#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
//#include <SparkFun_MMC5983MA_Arduino_Library.h>

#define I2C_BUS Wire

I2Cdev i2c_0(&I2C_BUS);

//SFE_MMC5983MA mag;
SparkFun_ISM330DHCX ism;
MMC5983MA mag(&i2c_0);

sfe_ism_data_t gyroData;
sfe_ism_data_t accelData;

uint32_t magData[3] = {0.0, 0.0, 0.0};

FusionVector gyroscope;
FusionVector accelerometer;
FusionVector magnetometer;

FusionAhrs ahrs;

FusionEuler attitude;

const float gyroOffset[3] = { 0.0024f, -0.0104f, -0.0043f };
const float accelOffset[3] = { -0.05773f, -0.01871f, 0.007735f };
float magOffset[3] = { 0.0, 0.0, 0.0 };
float magBias[3] = { -0.37f, 0.53f, 1.07f };
float magScale[3] = { 0.96, 1.13, 0.93 };

unsigned long t_last = 0;
float t_delta = 0;

void getMag(FusionVector* magnetometer) {
#define M magnetometer -> axis
	mag.readData(magData);
	M.x = ((float)magData[0] - 131072.0f) / 16384.0f - magBias[0];
	M.y = ((float)magData[1] - 131072.0f) / 16384.0f - magBias[1];
	M.z = ((float)magData[2] - 131072.0f) / 16384.0f - magBias[2];
	M.x *= magScale[0];
	M.y *= magScale[1];
	M.z *= -magScale[2];
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
	Wire.setClock(400000);

	i2c_0.I2Cscan();

	if (!ism.begin())//check if ism begins
	{
		Serial.println(F("ism did not begin"));
		while (1);
	}

	//reset ism
	ism.deviceReset();
	while (!ism.getDeviceReset())
	{
		delay(1);
	}
	Serial.println(F("ism reset"));

	/*mag.getOffset(magOffset);
	Serial.println(magOffset[0]);
	Serial.println(magOffset[1]);
	Serial.println(magOffset[2]);*/

	mag.reset();
	mag.SET();
	mag.init(MODR_200Hz, MBW_200Hz, MSET_100);
	mag.selfTest();

	/*
	mag.offsetBias(magBias, magScale);

	Serial.print(magBias[0]); Serial.print('\t');
	Serial.print(magBias[1]); Serial.print('\t');
	Serial.println(magBias[2]);

	Serial.print(magScale[0]); Serial.print('\t');
	Serial.print(magScale[1]); Serial.print('\t');
	Serial.println(magScale[2]);

	delay(5000);
	//*/

	ism.setDeviceConfig();
	ism.setBlockDataUpdate();

	// Set the output data rate and precision of the accelerometer
	ism.setAccelDataRate(ISM_XL_ODR_208Hz);
	ism.setAccelFullScale(ISM_4g);

	// Set the output data rate and precision of the gyroscope
	ism.setGyroDataRate(ISM_GY_ODR_208Hz);
	ism.setGyroFullScale(ISM_500dps);

	// Turn on the accelerometer's filter and apply settings.
	ism.setAccelFilterLP2();
	ism.setAccelSlopeFilter(ISM_LP_ODR_DIV_10);

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

	//*
	FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, t_delta);
	//*/FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, t_delta);
	attitude = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	//*
	Serial.print(attitude.angle.pitch); Serial.print('\t');
	Serial.print(attitude.angle.yaw); Serial.print('\t');
	Serial.print(attitude.angle.roll); Serial.print('\t');
	//*/
	delay(5);
	t_delta = (millis() - t_last) / 1000.0f;
	Serial.println(1.0f / t_delta);
}