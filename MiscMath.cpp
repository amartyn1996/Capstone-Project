#include "MiscMath.h"
#include "Arduino.h"

/**
 *  Takes in a vector and normalizes it.
 */
void MiscMath::normalize(float &X, float &Y, float &Z) {
  float len = vecLength(X, Y, Z);
  X /= len;
  Y /= len;
  Z /= len;
}

/**
 *  Returns the length of the input vector.
 */
float MiscMath::vecLength(float X, float Y, float Z) {
  return sqrt(X*X + Y*Y + Z*Z);
}
