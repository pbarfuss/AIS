#include <stdint.h>
#include <math.h>
#include "fast_atanf.h"

float fast_atanf(float z)
{
  float alpha, angle, base_angle, z_in = z;
  unsigned int index;

  if (z > 1.0f) {
    z = 1.0f/z;
  }

  /* when ratio approaches the table resolution, the angle is */
  /* best approximated with the argument itself... */
  if(z < TAN_MAP_RES) {
    base_angle = z;
  } else {
    /* find index and interpolation value */
    alpha = z * (float)TAN_MAP_SIZE;
    index = ((unsigned int)alpha) & 0xff;
    alpha -= (float)index;
    /* determine base angle based on quadrant and */
    /* add or subtract table value from base angle based on quadrant */
    base_angle  =  fast_atan_table[index];
    base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
  }

  if(z_in < 1.0f) { /* -PI/4 -> PI/4 or 3*PI/4 -> 5*PI/4 */
    angle = base_angle; /* 0 -> PI/4, angle OK */
  }
  else { /* PI/4 -> 3*PI/4 or -3*PI/4 -> -PI/4 */
    angle = (float)M_PI*0.5f - base_angle; /* PI/4 -> PI/2, angle = PI/2 - angle */
  }

  return (angle);
}

float polar_disc_fast(float ar, float aj, float br, float bj)
{
    float x = ar*br + aj*bj;
    float y = aj*br - ar*bj;
    float z;

    if ((y != y) || (x != x))
        return 0.0f;

    y += 1e-12f;
    if (x < 1e-12f) {
        z = (float)M_PI * 0.5f;
    } else {
        /* compute y/x */
        z=fast_atanf(fabsf(y/x));
        if (x < 0.0f) {
            z = (float)M_PI - z;
        }
    }

    if (z != z) {
        z = 0.0f;
    }

    if (y < 0.0f) {
        z = -z;
    }

    return (z * 0.31831f);
}

