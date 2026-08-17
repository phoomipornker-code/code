#ifndef UTIL_H
#define UTIL_H

/* The Saturation block. */
inline float clampf(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

#endif /* UTIL_H */
