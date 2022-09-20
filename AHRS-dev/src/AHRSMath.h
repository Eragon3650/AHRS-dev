// AHRSMath.h

#ifndef _AHRSMATH_h
#define _AHRSMATH_h

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
static inline bool VectorIsZero(const Vector vector);

static inline float VectorSum(const Vector vector);

static inline Vector VectorHadamardProduct(const Vector vectorA, const Vector vectorB);

static inline float VectorMagnitudeSquared(const Vector vector);
//vector math
static inline Vector VectorAdd(Vector vectorA, Vector vectorB);
static inline Vector VectorMultiplyScalar(Vector vector, float number);
static inline Vector VectorCrossProduct(Vector vectorA, Vector vectorB);
static inline Vector VectorNormalize(Vector vector);

static inline Quaternion QuaternionAdd(Quaternion quaternionA, Quaternion quaternionB);
static inline Quaternion QuaternionMultiplyScalar(Quaternion quaternion, float number);
static inline Quaternion QuaternionMultiply(Quaternion quaternionA, Quaternion quaternionB);
static inline Quaternion QuaternionMultiplyVector(Quaternion quaternion, Vector vector);
static inline Quaternion QuaternionConjugate(Quaternion quaternion);
static inline Quaternion QuaternionNormalize(Quaternion quaternion);
static inline Euler QuaternionToEuler(const Quaternion quaternion);
#endif
