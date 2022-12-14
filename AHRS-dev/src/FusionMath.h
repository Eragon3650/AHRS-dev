/**
 * @file FusionMath.h
 * @author Seb Madgwick
 * @brief Math library.
 */

#ifndef FUSION_MATH_H
#define FUSION_MATH_H

 //------------------------------------------------------------------------------
 // Includes

#include <math.h> // M_PI, sqrtf, atan2f, asinf
#include <stdbool.h>
#include <stdint.h>
#include <fastmath.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief 3D vector.
 */
typedef union {
	double array[3];

	struct {
		double x;
		double y;
		double z;
	} axis;
} FusionVector;

/**
 * @brief Quaternion.
 */
typedef union {
	double array[4];

	struct {
		double w;
		double x;
		double y;
		double z;
	} element;
} FusionQuaternion;

/**
 * @brief 3x3 matrix in row-major order.
 * See http://en.wikipedia.org/wiki/Row-major_order
 */
typedef union {
	double array[3][3];

	struct {
		double xx;
		double xy;
		double xz;
		double yx;
		double yy;
		double yz;
		double zx;
		double zy;
		double zz;
	} element;
} FusionMatrix;

/**
 * @brief Euler angles.  Roll, pitch, and yaw correspond to rotations around
 * X, Y, and Z respectively.
 */
typedef union {
	double array[3];

	struct {
		double roll;
		double pitch;
		double yaw;
	} angle;
} FusionEuler;

/**
 * @brief Vector of zeros.
 */
#define FUSION_VECTOR_ZERO ((FusionVector){ .array = {0.0, 0.0, 0.0} })

 /**
  * @brief Vector of ones.
  */
#define FUSION_VECTOR_ONES ((FusionVector){ .array = {1.0, 1.0, 1.0} })

  /**
   * @brief Identity quaternion.
   */
#define FUSION_IDENTITY_QUATERNION ((FusionQuaternion){ .array = {1.0, 0.0, 0.0, 0.0} })

   /**
	* @brief Identity matrix.
	*/
#define FUSION_IDENTITY_MATRIX ((FusionMatrix){ .array = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}} })

	/**
	 * @brief Euler angles of zero.
	 */
#define FUSION_EULER_ZERO ((FusionEuler){ .array = {0.0, 0.0, 0.0} })

	 /**
	  * @brief Pi. May not be defined in math.h.
	  */
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

	  /**
	   * @brief Include this definition or add as a preprocessor definition to use
	   * normal square root operations.
	   */
#define FUSION_USE_NORMAL_SQRT

	   //------------------------------------------------------------------------------
	   // Inline functions - Degrees and radians conversion

	   /**
		* @brief Converts degrees to radians.
		* @param degrees Degrees.
		* @return Radians.
		*/
static inline double FusionDegreesToRadians(const double degrees) {
	return degrees * ((double)M_PI / 180.0);
}

/**
 * @brief Converts radians to degrees.
 * @param radians Radians.
 * @return Degrees.
 */
static inline double FusionRadiansToDegrees(const double radians) {
	return radians * (180.0 / (double)M_PI);
}

//------------------------------------------------------------------------------
// Inline functions - Arc sine

/**
 * @brief Returns the arc sine of the value.
 * @param value Value.
 * @return Arc sine of the value.
 */
static inline double FusionAsin(const double value) {
	if (value <= -1.0f) {
		return (double)M_PI / -2.0;
	}
	if (value >= 1.0f) {
		return (double)M_PI / 2.0;
	}
	return asin(value);
}

//------------------------------------------------------------------------------
// Inline functions - Fast inverse square root

#ifndef FUSION_USE_NORMAL_SQRT

/**
 * @brief Calculates the reciprocal of the square root.
 * See https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/
 * @param x Operand.
 * @return Reciprocal of the square root of x.
 */
static inline double FusionFastInverseSqrt(const double x) {
	typedef union {
		double f;
		int32_t i;
	} Union32;

	Union32 union32 = { .f = x };
	union32.i = 0x5F1F1412 - (union32.i >> 1);
	return union32.f * (1.69000231f - 0.714158168f * x * union32.f * union32.f);
}

#endif

//------------------------------------------------------------------------------
// Inline functions - Vector operations

/**
 * @brief Returns true if the vector is zero.
 * @param vector Vector.
 * @return True if the vector is zero.
 */
static inline bool FusionVectorIsZero(const FusionVector vector) {
	return (vector.axis.x == 0.0) && (vector.axis.y == 0.0) && (vector.axis.z == 0.0);
}

/**
 * @brief Returns the sum of two vectors.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Sum of two vectors.
 */
static inline FusionVector FusionVectorAdd(const FusionVector vectorA, const FusionVector vectorB) {
	FusionVector result;
	result.axis.x = vectorA.axis.x + vectorB.axis.x;
	result.axis.y = vectorA.axis.y + vectorB.axis.y;
	result.axis.z = vectorA.axis.z + vectorB.axis.z;
	return result;
}

/**
 * @brief Returns vector B subtracted from vector A.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Vector B subtracted from vector A.
 */
static inline FusionVector FusionVectorSubtract(const FusionVector vectorA, const FusionVector vectorB) {
	FusionVector result;
	result.axis.x = vectorA.axis.x - vectorB.axis.x;
	result.axis.y = vectorA.axis.y - vectorB.axis.y;
	result.axis.z = vectorA.axis.z - vectorB.axis.z;
	return result;
}

/**
 * @brief Returns the sum of the elements.
 * @param vector Vector.
 * @return Sum of the elements.
 */
static inline double FusionVectorSum(const FusionVector vector) {
	return vector.axis.x + vector.axis.y + vector.axis.z;
}

/**
 * @brief Returns the multiplication of a vector by a scalar.
 * @param vector Vector.
 * @param scalar Scalar.
 * @return Multiplication of a vector by a scalar.
 */
static inline FusionVector FusionVectorMultiplyScalar(const FusionVector vector, const double scalar) {
	FusionVector result;
	result.axis.x = vector.axis.x * scalar;
	result.axis.y = vector.axis.y * scalar;
	result.axis.z = vector.axis.z * scalar;
	return result;
}

/**
 * @brief Calculates the Hadamard product (element-wise multiplication).
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Hadamard product.
 */
static inline FusionVector FusionVectorHadamardProduct(const FusionVector vectorA, const FusionVector vectorB) {
	FusionVector result;
	result.axis.x = vectorA.axis.x * vectorB.axis.x;
	result.axis.y = vectorA.axis.y * vectorB.axis.y;
	result.axis.z = vectorA.axis.z * vectorB.axis.z;
	return result;
}

/**
 * @brief Returns the cross product.
 * @param vectorA Vector A.
 * @param vectorB Vector B.
 * @return Cross product.
 */
static inline FusionVector FusionVectorCrossProduct(const FusionVector vectorA, const FusionVector vectorB) {
#define A vectorA.axis
#define B vectorB.axis
	FusionVector result;
	result.axis.x = A.y * B.z - A.z * B.y;
	result.axis.y = A.z * B.x - A.x * B.z;
	result.axis.z = A.x * B.y - A.y * B.x;
	return result;
#undef A
#undef B
}

/**
 * @brief Returns the vector magnitude squared.
 * @param vector Vector.
 * @return Vector magnitude squared.
 */
static inline double FusionVectorMagnitudeSquared(const FusionVector vector) {
	return FusionVectorSum(FusionVectorHadamardProduct(vector, vector));
}

/**
 * @brief Returns the vector magnitude.
 * @param vector Vector.
 * @return Vector magnitude.
 */
static inline double FusionVectorMagnitude(const FusionVector vector) {
	return sqrtf(FusionVectorMagnitudeSquared(vector));
}

/**
 * @brief Returns the normalised vector.
 * @param vector Vector.
 * @return Normalised vector.
 */
static inline FusionVector FusionVectorNormalise(const FusionVector vector) {
#ifdef FUSION_USE_NORMAL_SQRT
	const double magnitudeReciprocal = 1.0 / sqrt(FusionVectorMagnitudeSquared(vector));
#else
	const double magnitudeReciprocal = FusionFastInverseSqrt(FusionVectorMagnitudeSquared(vector));
#endif
	return FusionVectorMultiplyScalar(vector, magnitudeReciprocal);
}

//------------------------------------------------------------------------------
// Inline functions - Quaternion operations

/**
 * @brief Returns the sum of two quaternions.
 * @param quaternionA Quaternion A.
 * @param quaternionB Quaternion B.
 * @return Sum of two quaternions.
 */
static inline FusionQuaternion FusionQuaternionAdd(const FusionQuaternion quaternionA, const FusionQuaternion quaternionB) {
	FusionQuaternion result;
	result.element.w = quaternionA.element.w + quaternionB.element.w;
	result.element.x = quaternionA.element.x + quaternionB.element.x;
	result.element.y = quaternionA.element.y + quaternionB.element.y;
	result.element.z = quaternionA.element.z + quaternionB.element.z;
	return result;
}

/**
 * @brief Returns the multiplication of two quaternions.
 * @param quaternionA Quaternion A (to be post-multiplied).
 * @param quaternionB Quaternion B (to be pre-multiplied).
 * @return Multiplication of two quaternions.
 */
static inline FusionQuaternion FusionQuaternionMultiply(const FusionQuaternion quaternionA, const FusionQuaternion quaternionB) {
#define A quaternionA.element
#define B quaternionB.element
	FusionQuaternion result;
	result.element.w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z;
	result.element.x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y;
	result.element.y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x;
	result.element.z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w;
	return result;
#undef A
#undef B
}

/**
 * @brief Returns the multiplication of a quaternion with a vector.  This is a
 * normal quaternion multiplication where the vector is treated a
 * quaternion with a W element value of zero.  The quaternion is post-
 * multiplied by the vector.
 * @param quaternion Quaternion.
 * @param vector Vector.
 * @return Multiplication of a quaternion with a vector.
 */
static inline FusionQuaternion FusionQuaternionMultiplyVector(const FusionQuaternion quaternion, const FusionVector vector) {
#define Q quaternion.element
#define V vector.axis
	FusionQuaternion result;
	result.element.w = -Q.x * V.x - Q.y * V.y - Q.z * V.z;
	result.element.x = Q.w * V.x + Q.y * V.z - Q.z * V.y;
	result.element.y = Q.w * V.y - Q.x * V.z + Q.z * V.x;
	result.element.z = Q.w * V.z + Q.x * V.y - Q.y * V.x;
	return result;
#undef Q
#undef V
}

/**
 * @brief Returns the normalised quaternion.
 * @param quaternion Quaternion.
 * @return Normalised quaternion.
 */
static inline FusionQuaternion FusionQuaternionNormalise(const FusionQuaternion quaternion) {
#define Q quaternion.element
#ifdef FUSION_USE_NORMAL_SQRT
	const double magnitudeReciprocal = 1.0 / sqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
#else
	const double magnitudeReciprocal = FusionFastInverseSqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
#endif
	FusionQuaternion normalisedQuaternion;
	normalisedQuaternion.element.w = Q.w * magnitudeReciprocal;
	normalisedQuaternion.element.x = Q.x * magnitudeReciprocal;
	normalisedQuaternion.element.y = Q.y * magnitudeReciprocal;
	normalisedQuaternion.element.z = Q.z * magnitudeReciprocal;
	return normalisedQuaternion;
#undef Q
}

//------------------------------------------------------------------------------
// Inline functions - Matrix operations

/**
 * @brief Returns the multiplication of a matrix with a vector.
 * @param matrix Matrix.
 * @param vector Vector.
 * @return Multiplication of a matrix with a vector.
 */
static inline FusionVector FusionMatrixMultiplyVector(const FusionMatrix matrix, const FusionVector vector) {
#define R matrix.element
	FusionVector result;
	result.axis.x = R.xx * vector.axis.x + R.xy * vector.axis.y + R.xz * vector.axis.z;
	result.axis.y = R.yx * vector.axis.x + R.yy * vector.axis.y + R.yz * vector.axis.z;
	result.axis.z = R.zx * vector.axis.x + R.zy * vector.axis.y + R.zz * vector.axis.z;
	return result;
#undef R
}

//------------------------------------------------------------------------------
// Inline functions - Conversion operations

/**
 * @brief Converts a quaternion to a rotation matrix.
 * @param quaternion Quaternion.
 * @return Rotation matrix.
 */
static inline FusionMatrix FusionQuaternionToMatrix(const FusionQuaternion quaternion) {
#define Q quaternion.element
	const double qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
	const double qwqx = Q.w * Q.x;
	const double qwqy = Q.w * Q.y;
	const double qwqz = Q.w * Q.z;
	const double qxqy = Q.x * Q.y;
	const double qxqz = Q.x * Q.z;
	const double qyqz = Q.y * Q.z;
	FusionMatrix matrix;
	matrix.element.xx = 2.0 * (qwqw - 0.5 + Q.x * Q.x);
	matrix.element.xy = 2.0 * (qxqy - qwqz);
	matrix.element.xz = 2.0 * (qxqz + qwqy);
	matrix.element.yx = 2.0 * (qxqy + qwqz);
	matrix.element.yy = 2.0 * (qwqw - 0.5 + Q.y * Q.y);
	matrix.element.yz = 2.0 * (qyqz - qwqx);
	matrix.element.zx = 2.0 * (qxqz - qwqy);
	matrix.element.zy = 2.0 * (qyqz + qwqx);
	matrix.element.zz = 2.0 * (qwqw - 0.5 + Q.z * Q.z);
	return matrix;
#undef Q
}

/**
 * @brief Converts a quaternion to ZYX Euler angles in degrees.
 * @param quaternion Quaternion.
 * @return Euler angles in degrees.
 */
static inline FusionEuler FusionQuaternionToEuler(const FusionQuaternion quaternion) {
#define Q quaternion.element
	const double halfMinusQySquared = 0.5 - Q.y * Q.y; // calculate common terms to avoid repeated operations
	FusionEuler euler;
	euler.angle.roll = FusionRadiansToDegrees(atan2(Q.w * Q.x + Q.y * Q.z, halfMinusQySquared - Q.x * Q.x));
	euler.angle.pitch = FusionRadiansToDegrees(FusionAsin(2.0 * (Q.w * Q.y - Q.z * Q.x)));
	euler.angle.yaw = FusionRadiansToDegrees(atan2(Q.w * Q.z + Q.x * Q.y, halfMinusQySquared - Q.z * Q.z));
	return euler;
#undef Q
}

#endif

//------------------------------------------------------------------------------
// End of file
