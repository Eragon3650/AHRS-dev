/*
 Name:		AHRS_dev.ino
 Created:	9/17/22 23:05:32
 Author:	alexh
*/

// the setup function runs once when you press reset or power the board
#include "ism330dhcx_reg.h"
#include "ISM330DHCXSensor.h"
#include "Fusion.h"
#include "MMC5983MA.h"
#include "I2Cdev.h"
#include <Wire.h>

#define I2C_BUS Wire
I2Cdev i2c_0(&I2C_BUS);

#define SENSOR_ODR 416.0f
#define ACC_FS     8
#define GYR_FS     4000

ISM330DHCXSensor AccGyr(&I2C_BUS);
MMC5983MA mag(&i2c_0);

uint32_t magData[3]  = { 0, 0, 0 };
int32_t  gyroData[3] = { 0, 0, 0 };
int32_t accelData[3] = { 0, 0, 0 };

FusionVector gyroscope;
FusionVector accelerometer;
FusionVector magnetometer;

FusionAhrs ahrs;
FusionOffset offset;

FusionEuler attitude;

const int gyroOffset[3] = { 92, -551, -249 };
const int accelOffset[3] = { 4, -33, 9 };
float magBias[3] = { 0.05f, 0.28f, 1.03f };
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
	AccGyr.GYRO_GetAxes(gyroData);
	G.x = (gyroData[0] - gyroOffset[0]) / 1000.0f;
	G.y = (gyroData[1] - gyroOffset[1] ) / 1000.0f;
	G.z = (gyroData[2] - gyroOffset[2]) / 1000.0f;
#undef G
}

void getAccel(FusionVector* accel) {
#define A accel -> axis
    AccGyr.ACC_GetAxes(accelData);
	A.x = (accelData[0] - accelOffset[0] ) / 1000.0f;
	A.y = (accelData[1] - accelOffset[1] ) / 1000.0f;
	A.z = (accelData[2] - accelOffset[2] ) / 1000.0f;
#undef G
}

void setup() {
	Serial.begin(115200);

	Wire.begin();
	Wire.setClock(400000);

	i2c_0.I2Cscan();

	mag.reset();
	mag.SET();
	mag.init(MODR_1000Hz, MBW_100Hz, MSET_25);
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

    AccGyr.begin();

    //Accelerometer setup
    AccGyr.ACC_Enable();
    AccGyr.ACC_SetOutputDataRate(SENSOR_ODR);
    AccGyr.ACC_SetFullScale(ACC_FS);
    
    //Gyrometer setup
    AccGyr.GYRO_Enable();
    AccGyr.GYRO_SetOutputDataRate(SENSOR_ODR);
    AccGyr.GYRO_SetFullScale(GYR_FS);

	Serial.println(F("Starting..."));

	FusionAhrsInitialise(&ahrs);
}

// the loop function runs over and over again until power down or reset
void loop() {
	t_last = micros();
	getGyro(&gyroscope);
    Serial.print(micros() - t_last); Serial.print('\t');
	getAccel(&accelerometer);
    Serial.print(micros() - t_last); Serial.print('\t');

	FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, t_delta);
	Serial.print(micros() - t_last); Serial.print('\t');
	attitude = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	Serial.print(micros() - t_last); Serial.print('\t');
	/*
	Serial.print(attitude.angle.pitch); Serial.print('\t');
	Serial.print(attitude.angle.yaw); Serial.print('\t');
	Serial.print(attitude.angle.roll); Serial.print('\t');
	//*/
	delay(3.0 - (micros() - t_last) / 1000);
	t_delta = (micros() - t_last) / 1000000.0f;
	Serial.println(1.0f / t_delta);
}