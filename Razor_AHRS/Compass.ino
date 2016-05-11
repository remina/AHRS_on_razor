/* This file is part of the Razor AHRS Firmware */

void Compass_Heading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  float h_PI;
  
  h_PI = 0.5 * PI;
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated magnetic field X
  mag_x = magnetom_mean[0] * cos_pitch + magnetom_mean[1] * sin_roll * sin_pitch + magnetom_mean[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = magnetom_mean[2] * sin_roll - magnetom_mean[1] * cos_roll;
  // Magnetic Heading
  if (mag_x < 0 && mag_y < 0)
    MAG_Heading = -atan2(mag_y, mag_x) - PI;
  else if (mag_x < 0 && mag_y > 0)
    MAG_Heading = PI - atan2(mag_y, mag_x);
  else if (mag_x > 0 && mag_y > 0)
    MAG_Heading = PI - atan2(mag_y, mag_x);
  else
    MAG_Heading = -atan2(mag_y, mag_x) - PI;
}
