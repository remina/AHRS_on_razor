/* This file is part of the Razor AHRS Firmware */

// Calculate the scaled gyro readings in radians per second
float gyro_scaled_rad(float rate) 
{
  return(rate * TO_RAD(gyro_gain));
} 

//**********************************added new math functions******************************************************//
//float abselute
float fabs(float x)
{
  float out;
  if (x < 0)
    out = -x;
  else 
    out = x;
  return out;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void qua_norm(float* qua)
{
  float norm = invSqrt(qua[0] * qua[0] +  qua[1] * qua[1] + qua[2] * qua[2] + qua[3] * qua[3]);
  qua[0] *= norm;
  qua[1] *= norm;
  qua[2] *= norm;
  qua[3] *= norm;
}

//normlize vector
void norm(float *x, float *y, float *z)
{
  float norm = invSqrt(*x * *x + *y * *y + *z * *z);
  *x *= norm;
  *y *= norm;
  *z *= norm;
}

void qua_multiply(float from[4], float to[4], float *out)
{
  out[0] = from[0] * to[0] - from[1] * to[1] - from[2] * to[2] - from[3] * to[3];
  out[1] = from[0] * to[1] + from[1] * to[0] + from[2] * to[3] - from[3] * to[2];
  out[2] = from[0] * to[2] + from[2] * to[0] + from[3] * to[1] - from[1] * to[3];
  out[3] = from[0] * to[3] + from[3] * to[0] + from[1] * to[2] - from[2] * to[1];
}

// convert quatanion to euler
void qua_euler(void)
{
    pitch = asin(2 * qua[0] * qua[2] -2 * qua[1] * qua[3]);   
    roll  = atan2(2 * qua[2] * qua[3] + 2 * qua[0] * qua[1],-2 * qua[1] * qua[1] - 2 * qua[2] * qua[2] + 1);
    yaw   = atan2(2 * qua[1] * qua[2] + 2 * qua[0] * qua[3],-2 * qua[2] * qua[2] - 2 * qua[3] * qua[3] + 1);
}

//*****************************************************************************************************************************//

// Computes the dot product of two vectors
float Vector_Dot_Product(const float v1[3], const float v2[3])
{
  float result = 0;
  
  for(int c = 0; c < 3; c++)
  {
    result += v1[c] * v2[c];
  }
  
  return result; 
}

// Computes the cross product of two vectors
// out has to different from v1 and v2 (no in-place)!
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3])
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

// array calcu mean
float mean(float *m, float count)
{
	float sum = 0.0;
	for(char i = 0; i < count; i++)
		sum += *(m + i);
	return (sum/count);
}

// Adds two vectors
void Vector_Add(float out[3], const float v1[3], const float v2[3])
{
  for(int c = 0; c < 3; c++)
  {
    out[c] = v1[c] + v2[c];
  }
}

// Multiply two 3x3 matrices: out = a * b
// out has to different from a and b (no in-place)!
void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3])
{
  for(int x = 0; x < 3; x++)  // rows
  {
    for(int y = 0; y < 3; y++)  // columns
    {
      out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
    }
  }
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}
