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

#define I2C_BUS Wire
I2Cdev i2c_0(&I2C_BUS);

#define FREQUENCY 300

SparkFun_ISM330DHCX ism;
MMC5983MA mag(&i2c_0);

sfe_ism_data_t gyroData;
sfe_ism_data_t accelData;

uint32_t magData[3] = {0.0, 0.0, 0.0};

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

__attribute__((optimize("O3"))) void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}

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

	ism.setDeviceConfig();
	ism.setBlockDataUpdate();

	// Set the output data rate and precision of the accelerometer
	ism.setAccelDataRate(ISM_XL_ODR_416Hz);
	ism.setAccelFullScale(ISM_8g);

	// Set the output data rate and precision of the gyroscope
	ism.setGyroDataRate(ISM_GY_ODR_416Hz);
	ism.setGyroFullScale(ISM_2000dps);

	// Turn on the accelerometer's filter and apply settings.
	ism.setAccelFilterLP2();
	ism.setAccelSlopeFilter(ISM_HP_ODR_DIV_20);

	// Turn on the gyroscope's filter and apply settings.
	ism.setGyroFilterLP1();
	ism.setGyroLP1Bandwidth(ISM_STRONG);

	Serial.println(F("Starting..."));

	FusionOffsetInitialise(&offset, FREQUENCY);
	FusionAhrsInitialise(&ahrs);
}

// the loop function runs over and over again until power down or reset
void loop() {
	t_last = micros();
	getGyro(&gyroscope);
	getAccel(&accelerometer);

	//gyroscope = FusionOffsetUpdate(&offset, gyroscope);

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
	delay(3.0 - (micros() - t_last) / 1000);
	t_delta = (micros() - t_last) / 1000000.0f;
	Serial.println(1.0f / t_delta);
}