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

const int gyroOffset[3] = { 92, -551, -249 };
const int accelOffset[3] = { 4, -33, 9 };
float magOffset[3] = { 0.0, 0.0, 0.0 };
float magBias[3] = { 0.02f, 0.43f, 0.95f };
float magScale[3] = { 0.978f, 1.286f, 0.904 };

unsigned long t_last = 0;
float t_delta = 0;

void getMag(FusionVector *magnetometer) {
#define M magnetometer -> axis
	mag.readData(magData);
	M.x = ((float)magData[0] - 131072.0f) / 16384.0f - magBias[0];
	M.y = ((float)magData[1] - 131072.0f) / 16384.0f - magBias[1];
	M.z = ((float)magData[2] - 131072.0f) / 16384.0f - magBias[2];
	M.x *= magScale[0];
	M.y *= magScale[1];
	M.z *= magScale[2];
}

void getGyro(FusionVector *gyro) {
#define G gyro -> axis
	ism.getGyro(&gyroData);
	G.x = (gyroData.xData - gyroOffset[0] ) / 1000.0f;
	G.y = (gyroData.yData - gyroOffset[1] ) / 1000.0f;
	G.z = (gyroData.zData - gyroOffset[2] ) / 1000.0f;
#undef G
}

void getAccel(FusionVector* accel) {
#define A accel -> axis
	ism.getAccel(&accelData);
	A.x = (accelData.xData - accelOffset[0] ) / 1000.0f;
	A.y = (accelData.yData - accelOffset[1] ) / 1000.0f;
	A.z = (accelData.zData - accelOffset[2] ) / 1000.0f;
#undef G
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

	mag.reset();
	mag.SET();
	mag.init(MODR_1000Hz, MBW_200Hz, MSET_100);
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
	ism.setAccelDataRate(ISM_XL_ODR_6667Hz);
	ism.setAccelFullScale(ISM_4g);

	// Set the output data rate and precision of the gyroscope
	ism.setGyroDataRate(ISM_GY_ODR_6667Hz);
	ism.setGyroFullScale(ISM_500dps);

	// Turn on the accelerometer's filter and apply settings.
	ism.setAccelFilterLP2(ISM_HP_ODR_DIV_10);
	ism.setAccelSlopeFilter(ISM_HP_ODR_DIV_20);

	// Turn on the gyroscope's filter and apply settings.
	ism.setGyroFilterLP1(ISM_HP_FILTER_16mHz);
	ism.setGyroLP1Bandwidth(ISM_STRONG);

	Serial.println(F("Starting..."));

	FusionAhrsInitialise(&ahrs);
}

// the loop function runs over and over again until power down or reset
void loop() {
	t_last = micros();
	getGyro(&gyroscope);
	getAccel(&accelerometer);

	/*
	getMag(&magnetometer);

	FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, t_delta);
	//*/FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, t_delta);
	attitude = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	//*
	Serial.print(attitude.angle.pitch); Serial.print('\t');
	Serial.print(attitude.angle.yaw); Serial.print('\t');
	Serial.print(attitude.angle.roll); Serial.print('\t');
	//*/
	delay(5);
	t_delta = (micros() - t_last) / 1000000.0f;
	Serial.println(1.0f / t_delta);
}