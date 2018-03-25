/*
  Name:		Sketch1.ino
  Created:	3/1/2018 4:22:40 PM
  Author:	IDDO
*/

// the setup function runs once when you press reset or power the board
#include <MPU9250_RegisterMap.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <SparkFunMPU9250-DMP.h>
//#include <quaternionFilters.h>
//#include <MPU9250.h>

int mission = 1;
const int SYNCING = 1;
const int CHECK_CROSSROAD = 2;
const int MOVE_STRIGHT = 3;
const int MOVE_RIGHT = 4;
const int MOVE_LEFT = 5;

Servo myservo;

float maxMagX = 100000;
float minMagX;
float maxMagY;
float minMagY;
float maxMagZ;
float minMagZ;
float magX;
float magY;
float magZ;
float magX100;
float magY100;
float magZ100;
float angleNow;

double avrDistanceR = 0;
double alphaDistance = 0.5;

double timer;

int LMotorB = A0; //Left motor backwards
int LMotorF = A1; //Left motor forwards
int LMotorS = 5; //Left motor speed

int RMotorF = A2; //Right motor backwards
int RMotorB = A3; //Right motor forwards
int RMotorS = 3; //Right motor speed

int inputUSLeft = 9;
int outputUSLeft = 8;

int inputUSRight = 13;
int outputUSRight = 12;

boolean b = true;
boolean synced = false;

SoftwareSerial BTserial(10, 11);

MPU9250_DMP imu;

void setup() {
	myservo.attach(7);
	BTserial.begin(9600);
	Serial.begin(9600);
	pinMode(LMotorB, OUTPUT);
	pinMode(LMotorF, OUTPUT);
	pinMode(LMotorS, OUTPUT);
	pinMode(RMotorB, OUTPUT);
	pinMode(RMotorF, OUTPUT);
	pinMode(RMotorS, OUTPUT);

	pinMode(inputUSLeft, INPUT);
	pinMode(outputUSLeft, OUTPUT);
	pinMode(inputUSRight, INPUT);
	pinMode(outputUSRight, OUTPUT);

	//while (BTBTserial.available()==0)
	//{
	//	//BTBTserial.println("a0");
	//}
	// Call imu.begin() to verify communication and initialize
	if (imu.begin() != INV_SUCCESS)
	{
		while (1)
		{
			BTserial.println("Unable to communicate with MPU-9250");
			BTserial.println("Check connections, and try again.");
			BTserial.println();
			delay(5000);
		}
	}

	Serial.println("begin");

	// Use setSensors to turn on or off MPU-9250 sensors.
	// Any of the following defines can be combined:
	// INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
	// INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
	// Enable all sensors:
	BTserial.println(imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS));

	// Use setGyroFSR() and setAccelFSR() to configure the
	// gyroscope and accelerometer full scale ranges.
	// Gyro options are +/- 250, 500, 1000, or 2000 dps
	imu.setGyroFSR(2000); // Set gyro to 2000 dps
	// Accel options are +/- 2, 4, 8, or 16 g
	imu.setAccelFSR(2); // Set accel to +/-2g
	// Note: the MPU-9250's magnetometer FSR is set at
	// +/- 4912 uT (micro-tesla's)

	// setLPF() can be used to set the digital low-pass filter
	// of the accelerometer and gyroscope.
	// Can be any of the following: 188, 98, 42, 20, 10, 5
	// (values are in Hz).
	imu.setLPF(5); // Set LPF corner frequency to 5Hz

	// The sample rate of the accel/gyro can be set using
	// setSampleRate. Acceptable values range from 4Hz to 1kHz
	imu.setSampleRate(10); // Set sample rate to 10Hz

	// Likewise, the compass (magnetometer) sample rate can be
	// set using the setCompassSampleRate() function.
	// This value can range between: 1-100Hz
	imu.setCompassSampleRate(10); // Set mag rate to 10Hz
}

void loop() {
	switch (mission) {
	case(SYNCING):
		syncingMaze();
		break;
	case(CHECK_CROSSROAD):

		break;
	case(MOVE_STRIGHT):
		moveStraight();
		break;
	case(MOVE_RIGHT):

		break;
	case(MOVE_LEFT):

		break;
	}
}

double angle() {
	imu.update( UPDATE_COMPASS);
	magX = imu.calcMag(imu.mx);
	magY = imu.calcMag(imu.my);

	magX100 = ((magX - minMagX) / (maxMagX - minMagX)) - 0.5;
	magY100 = ((magY - minMagY) / (maxMagY - minMagY)) - 0.5;
	//   BTserial.println(String(magX100));
	//   BTserial.println(String(magY100));
	angleNow = atan2(magX100, magY100) * 180 / PI;
	//BTserial.println(String(angleNow));
	return angleNow;
}
double distance(int outputUS, int inputUS, double avrDistance)
{
	digitalWrite(outputUS, LOW);
	delayMicroseconds(2);
	digitalWrite(outputUS, HIGH);
	delayMicroseconds(10);
	digitalWrite(outputUS, LOW);
	double distance = pulseIn(inputUS, HIGH) / 5.8 / 10;
	avrDistance = alphaDistance * distance + avrDistance * (1 - alphaDistance);
	return avrDistance;
}

void syncingMaze() {
	if (imu.dataReady())
	{
		BTserial.println("Press 'S' to syncing the program");
		if (BTserial.available() && BTserial.read() == 'S') {
			BTserial.println("Syncing the program");
			if (maxMagX == 100000) {
				imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
				maxMagX = imu.calcMag(imu.mx);
				minMagX = imu.calcMag(imu.mx);
				maxMagY = imu.calcMag(imu.my);
				minMagY = imu.calcMag(imu.my);
				maxMagZ = imu.calcMag(imu.mz);
				minMagZ = imu.calcMag(imu.mz);
			}
			while (b) {
				timer = millis() + 7000;
				while (millis() < timer) {
					imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
					magX = imu.calcMag(imu.mx);
					magY = imu.calcMag(imu.my);
					magZ = imu.calcMag(imu.mz);
					if (magX > maxMagX)
						maxMagX = magX;
					if (magX < minMagX)
						minMagX = magX;
					if (magY > maxMagY)
						maxMagY = magY;
					if (magY < minMagY)
						minMagY = magY;
					if (magZ > maxMagZ)
						maxMagZ = magZ;
					if (magZ < minMagZ)
						minMagZ = magZ;
					//BTserial.println(String(magX) + " ," + String(magY) + " ," + String(magZ));
					//BTserial.println();
					analogWrite(RMotorS, 120);
					digitalWrite(RMotorF, HIGH);
				}
				digitalWrite(RMotorF, LOW);
				//printIMUData();
				delay(50);
				BTserial.println();
				BTserial.println();
				BTserial.println();
				BTserial.println();
				BTserial.println("Max x: " + String(maxMagX) + ", Min x: " + String(minMagX) + ", Max y: " + String(maxMagY) + ", Min y: " + String(minMagY) + ", Max z: " + String(maxMagZ) + ", Min z: " + String(minMagZ));
				b = false;
			}
			synced = true;
		}
		if (synced) {
		}
	}
	else {
		BTserial.println("Loading...");
	}
}
void moveStraight() {
	myservo.write(45);
	BTserial.println("Put the robot in the start of the maze, looking forword into the maze (parallel to the side wall). After you did it send 'G'");
	while (1) {
		if (BTserial.available() && BTserial.read() == 'G') {
			digitalWrite(RMotorF, HIGH);
			digitalWrite(LMotorF, HIGH);
			break;
		}
	}
	while (1) {
		//double upAngle = angle();
		if (distance(outputUSRight, inputUSRight, avrDistanceR) > 5) {
			if (distance(outputUSRight, inputUSRight, avrDistanceR) > 15) {
				int farDis;
				for (int i = 0; i < 10; i++) {
					farDis = distance(outputUSRight, inputUSRight, 0);
					farDis = farDis / 2;
				}
				if (farDis > 15) {
					mission = 2;
					break;
				}
			}
			analogWrite(LMotorS, 150);
			analogWrite(RMotorS, 80);
		}
		if (distance(outputUSRight, inputUSRight, avrDistanceR) < 5) {
			analogWrite(RMotorS, 130);
			analogWrite(LMotorS, 80);
		}
		if (5 == distance(outputUSRight, inputUSRight, avrDistanceR)) {
			analogWrite(LMotorS, 90);
			analogWrite(RMotorS, 90);
		}
		if (distance(outputUSLeft, inputUSLeft, 0) < 5) {
			double blockDis;
			for (int i = 0; i < 10; i++) {
				blockDis = distance(outputUSLeft, inputUSLeft, blockDis);
				blockDis = blockDis / 2;
			}
			if (blockDis < 5) {
				analogWrite(LMotorS, 0);
				analogWrite(RMotorS, 0);
				digitalWrite(RMotorF, LOW);
				digitalWrite(LMotorF, LOW);
				mission = 2;
				break;
			}

		}

	}


}






void printIMUData(void)
{
	// After calling update() the ax, ay, az, gx, gy, gz, mx,
	// my, mz, time, and/or temerature class variables are all
	// updated. Access them by placing the object. in front:

	// Use the calcAccel, calcGyro, and calcMag functions to
	// convert the raw sensor readings (signed 16-bit values)
	// to their respective units.
	float accelX = imu.calcAccel(imu.ax);
	float accelY = imu.calcAccel(imu.ay);
	float accelZ = imu.calcAccel(imu.az);
	float gyroX = imu.calcGyro(imu.gx);
	float gyroY = imu.calcGyro(imu.gy);
	float gyroZ = imu.calcGyro(imu.gz);
	float magX = imu.calcMag(imu.mx);
	float magY = imu.calcMag(imu.my);
	float magZ = imu.calcMag(imu.mz);

	BTserial.println("Accel: " + String(accelX) + ", " +
		String(accelY) + ", " + String(accelZ) + " g");
	BTserial.println("Gyro: " + String(gyroX) + ", " +
		String(gyroY) + ", " + String(gyroZ) + " dps");
	BTserial.println("Mag: " + String(magX) + ", " +
		String(magY) + ", " + String(magZ) + " uT");
	BTserial.println("Time: " + String(imu.time) + " ms");
	BTserial.println();
}
