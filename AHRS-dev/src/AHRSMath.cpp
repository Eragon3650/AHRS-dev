#include "AHRSMath.h"

static inline Vector VectorAdd(const Vector vectorA, const Vector vectorB) {
	Vector sum;
	sum.axis.x = vectorA.axis.x + vectorB.axis.x;
	sum.axis.y = vectorA.axis.y + vectorB.axis.y;
	sum.axis.z = vectorA.axis.z + vectorB.axis.z;
	return sum;
}

static inline Vector VectorMultiplyScalar(const Vector vector, const float number) {
	Vector product;
	product.axis.x = vector.axis.x * number;
	product.axis.y = vector.axis.y * number;
	product.axis.z = vector.axis.z * number;
	return product;
}

static inline Vector VectorCrossProduct(Vector vectorA, Vector vectorB) {
#define A vectorA.axis
#define B vectorA.axis
	Vector result;
	result.axis.x = A.y * B.z - A.z * B.y;
	result.axis.y = A.z * B.x - A.x * B.z;
	result.axis.z = A.x * B.y - A.y * B.x;
	return result;
#undef A
#undef B
}

static inline Vector VectorNormalize(Vector vector) {
#define V vector.axis
	Vector result;
	const float normal = 1.0f / sqrtf(V.x * V.x + V.y * V.y + V.z * V.z);
	result.axis.x = V.x / normal;
	result.axis.y = V.y / normal;
	result.axis.z = V.z / normal;
	return result;
#undef v
}

static inline Vector VectorHadamardProduct(const Vector vectorA, const Vector vectorB) {
	Vector result;
	result.axis.x = vectorA.axis.x * vectorB.axis.x;
	result.axis.y = vectorA.axis.y * vectorB.axis.y;
	result.axis.z = vectorA.axis.z * vectorB.axis.z;
	return result;
}

static inline float VectorMagnitudeSquared(const Vector vector) {
	return VectorSum(VectorHadamardProduct(vector, vector));
}

static inline bool VectorIsZero(const Vector vector) {
	return (vector.axis.x == 0.0f) && (vector.axis.y == 0.0f) && (vector.axis.z == 0.0f);
}

static inline float VectorSum(const Vector vector) {
	return vector.axis.x + vector.axis.y + vector.axis.z;
}

static inline Quaternion QuaternionAdd(Quaternion const quaternionA, Quaternion const quaternionB) {
	Quaternion sum;
	sum.element.w = quaternionA.element.w + quaternionB.element.w;
	sum.element.x = quaternionA.element.x + quaternionB.element.x;
	sum.element.y = quaternionA.element.y + quaternionB.element.y;
	sum.element.z = quaternionA.element.z + quaternionB.element.z;
	return sum;
}

static inline Quaternion QuaternionMultiplyScalar(const Quaternion quaternion, const float number) {
	Quaternion product;
	product.element.w = quaternion.element.w * number;
	product.element.x = quaternion.element.x * number;
	product.element.y = quaternion.element.y * number;
	product.element.z = quaternion.element.z * number;
	return product;
}

static inline Quaternion QuaternionMultiply(const Quaternion quaternionA, const Quaternion quaternionB) {
#define L quaternionA.element
#define R quaternionB.element
	Quaternion product;
	product.element.w = (L.w * R.w) - (L.x * R.x) - (L.y * R.y) - (L.z * R.z);
	product.element.x = (L.w * R.x) + (L.x * R.w) + (L.y * R.z) - (L.z * R.y);
	product.element.y = (L.w * R.y) - (L.x * R.z) + (L.y * R.w) + (L.z * R.x);
	product.element.z = (L.w * R.z) + (L.x * R.y) - (L.y * R.x) + (L.z * R.w);
	return product;
#undef L
#undef R
}

static inline Quaternion QuaternionMultiplyVector(const Quaternion quaternion, const Vector vector) {
#define Q quaternion.element
#define V vector.axis
	Quaternion result;
	result.element.w = -Q.x * V.x - Q.y * V.y - Q.z * V.z;
	result.element.x = Q.w * V.x + Q.y * V.z - Q.z * V.y;
	result.element.y = Q.w * V.y - Q.x * V.z + Q.z * V.x;
	result.element.z = Q.w * V.z + Q.x * V.y - Q.y * V.x;
	return result;
#undef Q
#undef V
}

static inline Quaternion QuaternionConjugate(const Quaternion quaternion) {
	Quaternion conjugate;
	conjugate.element.w = quaternion.element.w;
	conjugate.element.x = -quaternion.element.x;
	conjugate.element.y = -quaternion.element.y;
	conjugate.element.z = -quaternion.element.z;
	return conjugate;
}

//Returns normalized quaternion
static inline Quaternion QuaternionNormalize(const Quaternion quaternion) {
#define Q quaternion.element
	Quaternion result;
	const float normal = sqrtf(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
	result.element.w = Q.w / normal;
	result.element.x = Q.x / normal;
	result.element.y = Q.y / normal;
	result.element.z = Q.z / normal;
	return result;
#undef Q
}

static inline Euler QuaternionToEuler(const Quaternion quaternion) {
#define Q quaternion.element
	const float halfMinusQySquared = 0.5f - Q.y * Q.y; // calculate common terms to avoid repeated operations
	Euler euler;
	euler.angle.roll = TO_DEG * (atan2(Q.w * Q.x + Q.y * Q.z, halfMinusQySquared - Q.x * Q.x));
	euler.angle.pitch = TO_DEG * (asin(2.0f * (Q.w * Q.y - Q.z * Q.x)));
	euler.angle.yaw = TO_DEG * (atan2(Q.w * Q.z + Q.x * Q.y, halfMinusQySquared - Q.z * Q.z));
	return euler;
#undef Q
}