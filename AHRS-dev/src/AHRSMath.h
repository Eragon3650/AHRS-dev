// AHRSMath.h

#ifndef AHRSMATH_h
#define AHRSMATH_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "math.h"
#include "Constants.h"

typedef struct {
	float gain;
	float accelRejection;
	float magRejection;
	unsigned int rejectionTimeout;
} AhrsSettings;

typedef union {
	float array[3];

	struct {
		float x;
		float y;
		float z;
	} axis;
} Vector;

typedef union {
	float array[4];

	struct {
		float w;
		float x;
		float y;
		float z;
	} element;
} Quaternion;

typedef union {
	float array[3][3];

	struct {
		float xx;
		float xy;
		float xz;
		float yx;
		float yy;
		float yz;
		float zx;
		float zy;
		float zz;
	} element;
} Matrix;

typedef union {
	float array[3];

	struct {
		float pitch;
		float yaw;
		float roll;
	} angle;
} Euler;

const Vector VECTOR_ZERO = { 0.0f, 0.0f, 0.0f };
bool VectorIsZero(const Vector vector);

float VectorSum(const Vector vector);
Vector VectorHadamardProduct(const Vector vectorA, const Vector vectorB);
float VectorMagnitudeSquared(const Vector vector);
//vector math
Vector VectorAdd(Vector vectorA, Vector vectorB);
Vector VectorMultiplyScalar(Vector vector, float number);
Vector VectorCrossProduct(Vector vectorA, Vector vectorB);
Vector VectorNormalize(Vector vector);

Quaternion QuaternionAdd(Quaternion quaternionA, Quaternion quaternionB);
Quaternion QuaternionMultiplyScalar(Quaternion quaternion, float number);
Quaternion QuaternionMultiply(Quaternion quaternionA, Quaternion quaternionB);
Quaternion QuaternionMultiplyVector(Quaternion quaternion, Vector vector);
Quaternion QuaternionConjugate(Quaternion quaternion);
Quaternion QuaternionNormalize(Quaternion quaternion);
Euler QuaternionToEuler(const Quaternion quaternion);
#endif
