/* This file is part of the Razor AHRS Firmware */

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

//************************sometihng for quatanion************************************//
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

// Multiply the vector by a scalar
void Vector_Scale(float out[3], const float v[3], float scale)
{
  for(int c = 0; c < 3; c++)
  {
    out[c] = v[c] * scale; 
  }
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

// Init rotation matrix using euler angles
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll)
{
  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);

  // Euler angles, right-handed, intrinsic, XYZ convention
  // (which means: rotate around body axes Z, Y', X'') 
  m[0][0] = c2 * c3;
  m[0][1] = c3 * s1 * s2 - c1 * s3;
  m[0][2] = s1 * s3 + c1 * c3 * s2;

  m[1][0] = c2 * s3;
  m[1][1] = c1 * c3 + s1 * s2 * s3;
  m[1][2] = c1 * s2 * s3 - c3 * s1;

  m[2][0] = -s2;
  m[2][1] = c2 * s1;
  m[2][2] = c1 * c2;
}


// Init quatanion using euler angles
void init_quatanion(float m[4], float yaw, float pitch, float roll)
{
  float h_roll = 0.5 * roll;
  float h_pitch = 0.5 * pitch;
  float h_yaw = 0.5 * yaw;
  float c1 = cos(h_roll);
  float s1 = sin(h_roll);
  float c2 = cos(h_pitch);

  float s2 = sin(h_pitch);
  float c3 = cos(h_yaw);
  float s3 = sin(h_yaw);

  m[0] = c1 * c2 * c3 - s1 * s2 * s3;  
  m[1] = s1 * c2 * c3 + c1 * s2 * s3;  
  m[2] = c1 * s2 * c3 - s1 * c2 * s3;  
  m[3] = c1 * c2 * s3 + s1 * s2 * c3;       
}

// convert quatanion to euler
void qua_euler(void)
{
    pitch = -1.0f * asin(-2 * qua[1] * qua[3] + 2 * qua[0] * qua[2]);   
    roll  = atan2(2 * qua[2] * qua[3] + 2 * qua[0] * qua[1],-2 * qua[1] * qua[1] - 2 * qua[2] * qua[2] + 1);
    yaw   = atan2(2 * qua[1] * qua[2] + 2 * qua[0] * qua[3],-2 * qua[2] * qua[2] - 2 * qua[3] * qua[3] + 1);
}



