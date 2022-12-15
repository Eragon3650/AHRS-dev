/*
 Name:		AHRS_dev.ino
 Created:	9/17/22 23:05:32
 Author:	alexh
*/

// the setup function runs once when you press reset or power the board
#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include "imx_rt1060/imx_rt1060_i2c_driver.h"
#include "Fusion.h"
#include "MMC5983MA_src/SparkFun_MMC5983MA_Arduino_Library.h"
#include <SPI.h>
#include "ISM330DHCX_src/SparkFun_ISM330DHCX.h"
#include "GPS_src/Adafruit_GPS.h"

#define GPSSerial Serial4
Adafruit_GPS GPS(&GPSSerial);

bool gpsStatus = false;

//Adafruit_BMP3XX bmp;

#define SEALEVELPRESSURE_INHG (29.95)
#define ALTIMETER_CALIBRATE_TIME (5000)
#define CUSTOM_ALTITUDE_CALC (false)

const float SEALEVELPRESSURE_HPA = SEALEVELPRESSURE_INHG * 33.86389f;

float initHeight = 0.0f;
float altitudeAdjusted = 0.0f;
float heightAdjusted = 0.0f;

SparkFun_ISM330DHCX ism;

FusionVector gyroscope;
FusionVector accelerometer;

const int gyroOffset[3] = { 92, -551, -249 };
const int accelOffset[3] = { 4, -33, 9 };

#define USE_MAG

#ifdef USE_MAG
SFE_MMC5983MA mag;

const double magBias[3] = { 0.05, 0.28, 1.03 };
const double magScale[3] = { 0.978, 1.286, 0.904 };

FusionVector magnetometer;
#endif

FusionAhrs ahrs;
FusionOffset offset;

#define METER_TO_FEET (3.2808395)
#define FREQUENCY (416)

FusionEuler attitude;

elapsedMillis altimeterTimer;
elapsedMillis serialTimer;
elapsedMicros executeTime;

double t_delta = 0;

/*
void altimeterCalibrate(int t) {
	Serial.println(F("Calibrating Altimeter..."));
	elapsedMillis timer1;
	elapsedMillis timer2;
	int numSamples = 0;
	float tempHeight;
	initHeight = 0;

	while (timer1 < t) {
		if (timer2 >= 10) {
			timer2 = 0;
			tempHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA);
			
			if (tempHeight < 1600.0) {
				initHeight += tempHeight;
				numSamples++;
			}
		}
	}
	
	initHeight /= (float)numSamples;
	Serial.println(F("Calibration complete"));
	Serial.print(F("Starting altitude: ")); Serial.println(initHeight);
}
//*/

#ifdef USE_MAG
void getMag(FusionVector *magnetometer) {
#define M magnetometer -> axis
	uint32_t magDataX = 0;
	uint32_t magDataY = 0;
	uint32_t magDataZ = 0;
	mag.readFieldsXYZ(&magDataX, &magDataY, &magDataZ);
	M.x = ((double)magDataX - 131072.0) / 131072.0 - magBias[0];
	M.y = ((double)magDataY - 131072.0) / 131072.0 - magBias[1];
	M.z = ((double)magDataZ - 131072.0) / 131072.0 - magBias[2];
	M.x *= magScale[0];
	M.y *= magScale[1];
	M.z *= magScale[2];
}
#endif

void getGyro(FusionVector *gyro) {
#define G gyro -> axis
	sfe_ism_data_t gyroData;
	ism.getGyro(&gyroData);
	G.x = (gyroData.xData - gyroOffset[0] ) * 0.001;
	G.y = (gyroData.yData - gyroOffset[1] ) * 0.001;
	G.z = (gyroData.zData - gyroOffset[2] ) * 0.001;
#undef G
}

void getAccel(FusionVector* accel) {
#define A accel -> axis
	sfe_ism_data_t accelData;
	ism.getAccel(&accelData);
	A.x = (accelData.xData - accelOffset[0] ) * 0.001;
	A.y = (accelData.yData - accelOffset[1] ) * 0.001;
	A.z = (accelData.zData - accelOffset[2] ) * 0.001;
#undef A
}

void setup() {
	Serial.begin(115200);

	Wire.setClock(400000);
	Wire.begin();

	//*
	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_BAUD_9600);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);// 1 Hz update rate
	// For the parsing code to work nicely and have time to sort thru the data, and
	// print it out we don't suggest using anything higher than 1 Hz

	// Request updates on antenna status, comment out to keep quiet
	GPS.sendCommand(PGCMD_ANTENNA);
	GPS.sendCommand(PMTK_ENABLE_SBAS);
	GPS.sendCommand(PMTK_ENABLE_WAAS);
	delay(1000);

	// Ask for firmware version
	GPSSerial.println(PMTK_Q_RELEASE);
	//*/

	/*
	if (!bmp.begin_I2C()) {
		Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
	}
	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	bmp.setOutputDataRate(BMP3_ODR_50_HZ);

	altimeterCalibrate(ALTIMETER_CALIBRATE_TIME);
	*/

#ifdef USE_MAG

	if (!mag.begin())//check if ism begins
	{
		Serial.println(F("mag did not begin"));
		while (1);
	}

	mag.softReset();

	Serial.println(F("MMC5983MA connected"));

	mag.setFilterBandwidth(800);

	mag.setContinuousModeFrequency(100);

	mag.enableAutomaticSetReset();

	mag.enableContinuousMode();

#endif

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

	ism.setDeviceConfig();
	ism.setBlockDataUpdate();

	// Set the output data rate and precision of the accelerometer
	ism.setAccelDataRate(ISM_XL_ODR_416Hz);
	ism.setAccelFullScale(ISM_16g);

	// Set the output data rate and precision of the gyroscope
	ism.setGyroDataRate(ISM_GY_ODR_416Hz);
	ism.setGyroFullScale(ISM_2000dps);

	// Turn on the accelerometer's filter and apply settings.
	ism.setAccelFilterLP2();
	ism.setAccelSlopeFilter(ISM_HP_ODR_DIV_100);

	// Turn on the gyroscope's filter and apply settings.
	ism.setGyroFilterLP1();
	ism.setGyroLP1Bandwidth(ISM_STRONG);

	Serial.println(F("Starting..."));

	FusionOffsetInitialise(&offset, FREQUENCY);
	FusionAhrsInitialise(&ahrs);
}

// the loop function runs over and over again until power down or reset
void loop() {
	//*
	char c = GPS.read();
	if (GPS.newNMEAreceived()) {
		if (!GPS.parse(GPS.lastNMEA())) return;
		gpsStatus = GPS.fix;
	}
	//*/

	if (executeTime >= 800) {
		t_delta = executeTime * 0.000001;
		executeTime = 0;
		getGyro(&gyroscope);
		getAccel(&accelerometer);
		gyroscope = FusionOffsetUpdate(&offset, gyroscope);

#ifdef USE_MAG
		getMag(&magnetometer);

		FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, t_delta);
#else
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, t_delta);
#endif
		attitude = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	}

	if (serialTimer >= 40) {
		serialTimer = 0;

		Serial.print(1.0 / t_delta); Serial.print('\t');

		Serial.print(attitude.angle.pitch, 1); Serial.print('\t');
		Serial.print(attitude.angle.yaw, 1); Serial.print('\t');
		Serial.print(attitude.angle.roll, 1); Serial.print('\t');

		/*
		altitudeAdjusted = bmp.readAltitude(SEALEVELPRESSURE_HPA);
		heightAdjusted = METER_TO_FEET * (altitudeAdjusted - initHeight);
		Serial.print(altitudeAdjusted); Serial.print(F('\t'));
		Serial.print(heightAdjusted, 1);
		//*/

		//*
		if (gpsStatus) {
			Serial.print(GPS.latitudeDegrees, 6); Serial.print(F(", "));
			Serial.print(GPS.longitudeDegrees, 6); Serial.print('\t');
			Serial.print(GPS.satellites); Serial.print('\t');
		}
		//*/

		Serial.println();
	}
}