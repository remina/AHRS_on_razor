/* This file is part of the Razor AHRS Firmware */

//******************************newly added for AHRS alglorithm****************************************//
void error_calaulate(void)
{
  float hx = 0.0, hy = 0.0, bx = 0.0, bz = 0.0;
  // Auxiliary variables to avoid repeated arithmetic
  q0q1 = qua[0] * qua[1]; 
  q0q2 = qua[0] * qua[2]; 
  q0q3 = qua[0] * qua[3]; 
  q1q1 = qua[1] * qua[1]; 
  q1q2 = qua[1] * qua[2]; 
  q1q3 = qua[1] * qua[3]; 
  q2q2 = qua[2] * qua[2]; 
  q2q3 = qua[2] * qua[3]; 
  q3q3 = qua[3] * qua[3];

  ///////////////////////////////////////////////////////
  // Serial.print("#corrected & normlized accel:");
  // Serial.print(accel[0]);Serial.print(",");
  // Serial.print(accel[1]);Serial.print(",");
  // Serial.print(accel[2]);Serial.println();
  /////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////
  // Serial.print("#corrected & normlized magnetom:");
  // Serial.print(magnetom[0]);Serial.print(",");
  // Serial.print(magnetom[1]);Serial.print(",");
  // Serial.print(magnetom[2]);Serial.println();
  /////////////////////////////////////////////////////////

  // Reference direction of Earth's magnetic field
  hy = 2.0f * (magnetom[0] * (0.5f - q2q2 - q3q3) + magnetom[1] * (q1q2 - q0q3) + magnetom[2] * (q1q3 + q0q2));
  hx = 2.0f * (magnetom[0] * (q1q2 + q0q3) + magnetom[1] * (0.5f - q1q1 - q3q3) + magnetom[2] * (q2q3 - q0q1));
  bx = -1.0f * (sqrt((hx * hx) + (hy * hy)));
  bz = 2.0f * (magnetom[0] * (q1q3 - q0q2) + magnetom[1] * (q2q3 + q0q1) + magnetom[2] * (0.5f - q1q1 - q2q2));

  ///////////////////////////////////////////////////////////////
  // Serial.print("#estimated magnetom:");
  // Serial.print(hx);Serial.print(",");
  // Serial.print(hy);Serial.print(",");
  // Serial.print(bx);Serial.print(",");
  // Serial.print(bz);Serial.println();
  //////////////////////////////////////////////////////////////////

  // Estimated direction of gravity 
  vx = 2.0 * (q0q2 - q1q3);
  vy = -2.0 * (q0q1 + q2q3);
  vz = 2.0 * (0.5f - q1q1 - q2q2);

  //normlize estimated output
  norm(&vx, &vy, &vz);

  ///////////////////////////////////////////////////////
  // Serial.print("#final estimated gravity:");
  // Serial.print(vx);Serial.print(",");
  // Serial.print(vy);Serial.print(",");
  // Serial.print(vz);Serial.println();
  /////////////////////////////////////////////////////////
  // Estimated direction of magnetic 
     if ((magnetom[0] * magnetom[1]) > 0)
     {
	   wy =  2.0 * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
	   wx =  -2.0 * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
	   wz =  2.0 * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));  
     }
     else
     {
	   wy =  -2.0 * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
	   wx =  2.0 * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
	   wz =  2.0 * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2)); 
     }
  // else if (magnetom[0] > 0 && magnetom[1] < 0)
  // {
	// wy =  -2.0 * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
	// wx =  2.0 * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
	// wz =  2.0 * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));   
  // }
  // else
  // {
	// wy =  2.0 * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
	// wx =  -2.0 * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
	// wz =  2.0 * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));  
  // }	


  //normlize estimated output
  norm(&wx, &wy, &wz);

  ///////////////////////////////////////////////////////
  // Serial.print("#final estimated magnetom:");
  // Serial.print(wx);Serial.print(",");
  // Serial.print(wy);Serial.print(",");
  // Serial.print(wz);Serial.println();
  /////////////////////////////////////////////////////////
  // Error is sum of cross product between estimated direction and measured direction of field vectors
  ex = (accel[1] * vz - accel[2] * vy) + (magnetom[1] * wz - magnetom[2] * wy);
  ey = (accel[2] * vx - accel[0] * vz) + (magnetom[2] * wx - magnetom[0] * wz);
  ez = (accel[0] * vy - accel[1] * vx) + (magnetom[0] * wy - magnetom[1] * wx); 
  if(fabs(ex) < 0.01) ex = 0.0f;
  if(fabs(ey) < 0.01) ey = 0.0f;
  if(fabs(ez) < 0.01) ez = 0.0f;
  ///////////////////////////////////////////////////////
  Serial.print("#errors:");
  Serial.print(ex);Serial.print(",");
  Serial.print(ey);Serial.print(",");
  Serial.print(ez);Serial.println();
  /////////////////////////////////////////////////////////
}

void quatanion_update(void)
{
  // corrected gyro
  float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;

  ///////////////////////////////////////////////////////
  // Serial.print("#PI parameters:");
  // Serial.print(two_kp);Serial.print(",");
  // Serial.print(two_ki);Serial.println();
  /////////////////////////////////////////////////////////
  
  // Compute and apply integral feedback if enabled(?2?3????PIDT?yуюб?Y?)
  if(ex != 0.0f && ey != 0.0f && ez != 0.0f) 
  {
   integral_x += ex ;  // integral error scaled by Ki
   integral_y += ey ;
   integral_z += ez ;
  }
  else
  {
    integral_x = 0.0f;  // prevent integral gyroindup
    integral_y = 0.0f;
    integral_z = 0.0f;
  }
  ///////////////////////////////////////////////////////
  // Serial.print("#raw gyro in rad:");
  // Serial.print(ripe_gyro[0]);Serial.print(",");
  // Serial.print(ripe_gyro[1]);Serial.print(",");
  // Serial.print(ripe_gyro[2]);Serial.println();
  /////////////////////////////////////////////////////////
  for (char i = 0; i < 3; i++)
  {
	  if (fabs(ripe_gyro[i]) < 0.50)
		  ripe_gyro[i] = 0.0;
  }
  /////////////////////////////////////////////////////////
  // apply p & i
  gyro_x = ripe_gyro[0] - two_kp * ex - two_ki * integral_x;
  gyro_y = ripe_gyro[1] - two_kp * ey - two_ki * integral_y;
  gyro_z = ripe_gyro[2] - two_kp * ez - two_ki * integral_z;

  ///////////////////////////////////////////////////////
  // Serial.print("#integral errors:");
  // Serial.print(integral_x);Serial.print(",");
  // Serial.print(integral_y);Serial.print(",");
  // Serial.print(integral_z);Serial.println();
  /////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////
  Serial.print("#corrected gyro:");
  Serial.print(gyro_x);Serial.print(",");
  Serial.print(gyro_y);Serial.print(",");
  Serial.print(gyro_z);Serial.println();
  /////////////////////////////////////////////////////////
  //using quatanion differential to update
  //quatanion interation
  ///////////////////////////////////////////////////////
  Serial.print("#interation time:");
  Serial.print(G_Dt);Serial.println();
  /////////////////////////////////////////////////////////
  qua[0] += (-qua[1] * gyro_x - qua[2] * gyro_y + qua[3] * gyro_z) * G_Dt / 2.0f;
  qua[1] += (qua[0] * gyro_x - qua[2] * gyro_z - qua[3] * gyro_y) *  G_Dt / 2.0f;
  qua[2] += (qua[0] * gyro_y + qua[1] * gyro_z + qua[3] * gyro_x) *  G_Dt / 2.0f;
  qua[3] += (-qua[0] * gyro_z + qua[1] * gyro_y - qua[2] * gyro_x) *  G_Dt / 2.0f;
  qua_norm(qua); 
  //////////////////////////////////////////////////////////
  // Serial.print("#quatanion after:");
  Serial.print(qua[0]);Serial.print(",");
  Serial.print(qua[1]);Serial.print(",");
  Serial.print(qua[2]);Serial.print(",");
  Serial.print(qua[3]);Serial.println();
  //////////////////////////////////////////////////////////////
  // convert quatanion to euler
  qua_euler();
  Serial.print("#YPR=");
  Serial.print(TO_DEG(yaw)); Serial.print(",");
  Serial.print(TO_DEG(pitch)); Serial.print(",");
  Serial.print(TO_DEG(roll)); Serial.println();
}





