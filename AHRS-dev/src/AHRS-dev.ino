/*
 Name:		AHRS_dev.ino
 Created:	9/17/22 23:05:32
 Author:	alexh
*/

// the setup function runs once when you press reset or power the board
#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include "Fusion.h"
#include "MMC5983MA_src/SparkFun_MMC5983MA_Arduino_Library.h"
#include "ISM330DHCX_src/SparkFun_ISM330DHCX.h"
#include <SPI.h>
#include <RH_RF95.h>
#include "GPS_src/Adafruit_GPS.h"
#include "BMP_src/BMP3XX.h"
#include <PWMServo.h>

PWMServo myservo;  // create servo object to control a servo

RH_RF95 rf95(10, digitalPinToInterrupt(14));

#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);

bool gpsStatus = false;

BMP3XX bmp;

#define SEALEVELPRESSURE_INHG (30.21)

const float SEALEVELPRESSURE_HPA = SEALEVELPRESSURE_INHG * 33.86389f;

SparkFun_ISM330DHCX ism;

FusionVector gyroscope;
FusionVector accelerometer;

const int gyroOffset[3] = { 92, -551, -249 };
const int accelOffset[3] = { 4, -33, 9 };

//#define USE_MAG

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

elapsedMicros serialTimer;
elapsedMicros executeTime;

double t_delta = 0;

byte radioData[50];

#ifdef USE_MAG
void getMag(FusionVector* magnetometer) {
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

void getGyro(FusionVector* gyro) {
#define G gyro -> axis
	sfe_ism_data_t gyroData;
	ism.getGyro(&gyroData);
	G.x = (gyroData.xData - gyroOffset[0]) * 0.001;
	G.y = (gyroData.yData - gyroOffset[1]) * 0.001;
	G.z = (gyroData.zData - gyroOffset[2]) * 0.001;
#undef G
}

void getAccel(FusionVector* accel) {
#define A accel -> axis
	sfe_ism_data_t accelData;
	ism.getAccel(&accelData);
	A.x = (accelData.xData - accelOffset[0]) * 0.001;
	A.y = (accelData.yData - accelOffset[1]) * 0.001;
	A.z = (accelData.zData - accelOffset[2]) * 0.001;
#undef A
}

void setup() {
	myservo.attach(2);
	myservo.write(85); //105
	Serial.begin(115200);

	Wire.begin();
	Wire.setClock(400000);

	Wire1.setClock(1000000);
	Wire1.begin();

	//*
	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_BAUD_9600);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);// 1 Hz update rate
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	// For the parsing code to work nicely and have time to sort thru the data, and
	// print it out we don't suggest using anything higher than 1 Hz

	// Request updates on antenna status, comment out to keep quiet
	GPS.sendCommand(PGCMD_ANTENNA);
	GPS.sendCommand(PMTK_ENABLE_SBAS);
	GPS.sendCommand(PMTK_ENABLE_WAAS);
	delay(1000);

	// Ask for firmware version
	//GPSSerial.println(PMTK_Q_RELEASE);
	//*/
	if (!bmp.begin_I2C((uint8_t)119, &Wire1)) {
			Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
	}

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

			//*/

			if (!ism.begin(Wire))//check if ism begins
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
			ism.setGyroLP1Bandwidth(ISM_MEDIUM);
			//*/

			if (!rf95.init()) Serial.println(F("RF95 init failed"));
			if (!rf95.setFrequency(915.0)) Serial.println(F("Set frequency failed"));
			rf95.setTxPower(20, false);
			/*
			rf95.setSpreadingFactor(6);
			rf95.setSignalBandwidth(250001);
			//*/

			Serial.println(F("Starting..."));

			FusionOffsetInitialise(&offset, FREQUENCY);
			FusionAhrsInitialise(&ahrs);
		}

	// the loop function runs over and over again until power down or reset
	void loop() {
		//*
		char c = GPS.read();
		if (GPS.newNMEAreceived()) {
			gpsStatus = GPS.fix;
			if (!GPS.parse(GPS.lastNMEA())) return;
		}
		//*/

		if (executeTime >= 300) {
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

		myservo.write(85 - (abs(attitude.angle.pitch) * 6.5 / 9.0));

		if (serialTimer >= 1000000) {
			serialTimer = 0;

			//*
			Serial.print(attitude.angle.pitch, 1); Serial.print('\t');
			Serial.print(attitude.angle.yaw, 1); Serial.print('\t');
			Serial.print(attitude.angle.roll, 1); Serial.print('\t');

			Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)); Serial.print('\t');

			if (gpsStatus) {
				Serial.print(GPS.latitudeDegrees, 6); Serial.print(F(", "));
				Serial.print(GPS.longitudeDegrees, 6); Serial.print('\t');
				Serial.print(GPS.satellites); Serial.print('\t');
			}
			//*/
			double lat = 0.0;
			double lon = 0.0;
			if (gpsStatus) {
				lat = GPS.latitudeDegrees;
				lon = GPS.longitudeDegrees;
			}

			//*

			String temp_radioData = String(attitude.angle.pitch, 1) + "," +
				String(attitude.angle.roll, 1) + "," +
					String(attitude.angle.yaw, 1) + "," +
						String(bmp.readAltitude(SEALEVELPRESSURE_HPA), 2) + "," +
							String(lat, 6) + "," +
								String(lon, 6) + "," +
									"\n";

			temp_radioData.getBytes(radioData, sizeof(radioData));

			rf95.send(radioData, sizeof(radioData));
			//*/

			Serial.println(serialTimer);
		}
	}