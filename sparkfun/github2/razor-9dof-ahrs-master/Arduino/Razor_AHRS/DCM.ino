/* This file is part of the Razor AHRS Firmware */

// DCM algorithm

/**************************************************/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}

void Matrix_update(void)
{
  Gyro_Vector[0]=GYRO_SCALED_RAD(gyro[0]); //gyro x roll
  Gyro_Vector[1]=GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
  Gyro_Vector[2]=GYRO_SCALED_RAD(gyro[2]); //gyro z yaw
  
  Accel_Vector[0]=accel[0];
  Accel_Vector[1]=accel[1];
  Accel_Vector[2]=accel[2];
    
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
  
#if DEBUG__NO_DRIFT_CORRECTION == true // Do not use drift correction
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
#else // Use drift correction
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
#endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

void Euler_angles(void)
{
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}


//*****************************quatanion algorithm**************************************//
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
  Serial.print("#corrected & normlized accel:");
  Serial.print(accel[0]);Serial.print(",");
  Serial.print(accel[1]);Serial.print(",");
  Serial.print(accel[2]);Serial.println();
  /////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////
  Serial.print("#corrected & normlized magnetom:");
  Serial.print(magnetom[0]);Serial.print(",");
  Serial.print(magnetom[1]);Serial.print(",");
  Serial.print(magnetom[2]);Serial.println();
  /////////////////////////////////////////////////////////

  // Reference direction of Earth's magnetic field
  hx = 2.0f * (magnetom[0] * (0.5f - q2q2 - q3q3) + magnetom[1] * (q1q2 - q0q3) + magnetom[2] * (q1q3 + q0q2));
  hy = 2.0f * (magnetom[0] * (q1q2 + q0q3) + magnetom[1] * (0.5f - q1q1 - q3q3) + magnetom[2] * (q2q3 - q0q1));
  bx = -1.0f * (sqrt((hx * hx) + (hy * hy)));
  bz = 2.0f * (magnetom[0] * (q1q3 - q0q2) + magnetom[1] * (q2q3 + q0q1) + magnetom[2] * (0.5f - q1q1 - q2q2));

  ///////////////////////////////////////////////////////////////
  Serial.print("#estimated magnetom:");
  Serial.print(hx);Serial.print(",");
  Serial.print(hy);Serial.print(",");
  Serial.print(bx);Serial.print(",");
  Serial.print(bz);Serial.println();
  //////////////////////////////////////////////////////////////////

  // Estimated direction of gravity 
  vx = 2.0 * (q0q2 - q1q3);
  vy = 2.0 * (q0q1 + q2q3);
  vz = 2.0 * (0.5f - q1q1 - q2q2);

  //normlize estimated output
  norm(&vx, &vy, &vz);

  ///////////////////////////////////////////////////////
  Serial.print("#final estimated gravity:");
  Serial.print(vx);Serial.print(",");
  Serial.print(vy);Serial.print(",");
  Serial.print(vz);Serial.println();
  /////////////////////////////////////////////////////////
  // Estimated direction of magnetic 
  wx =  2.0 * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
  wy =  -2.0 * (bx * (q1q2 - q0q3) - bz * (q0q1 + q2q3));
  wz =  -2.0 * (bx * (q0q2 + q1q3) - bz * (0.5f - q1q1 - q2q2));

  //normlize estimated output
  norm(&wx, &wy, &wz);

  ///////////////////////////////////////////////////////
  Serial.print("#final estimated magnetom:");
  Serial.print(wx);Serial.print(",");
  Serial.print(wy);Serial.print(",");
  Serial.print(wz);Serial.println();
  /////////////////////////////////////////////////////////
  // Error is sum of cross product between estimated direction and measured direction of field vectors
  ex = (accel[1] * vz - accel[2] * vy) + (magnetom[1] * wz - magnetom[2] * wy);
  ey = (accel[2] * vx - accel[0] * vz) + (magnetom[2] * wx - magnetom[0] * wz);
  ez = (accel[0] * vy - accel[1] * vx) + (magnetom[0] * wy - magnetom[1] * wx); 
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
  Serial.print("#PI parameters:");
  Serial.print(two_kp);Serial.print(",");
  Serial.print(two_ki);Serial.println();
  /////////////////////////////////////////////////////////
  
  // Compute and apply integral feedback if enabled(?2?3????PIDT?y眢?Y?)
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

  // apply p & i
  gyro_x = gyro[0] + two_kp * ex + two_ki * integral_x;
  gyro_y = gyro[1] + two_kp * ey + two_ki * integral_y;
  gyro_z = gyro[2] + two_kp * ez + two_ki * integral_z;

  ///////////////////////////////////////////////////////
  Serial.print("#integral errors:");
  Serial.print(integral_x);Serial.print(",");
  Serial.print(integral_y);Serial.print(",");
  Serial.print(integral_z);Serial.println();
  /////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////
  Serial.print("#real gyro:");
  Serial.print(gyro[0]);Serial.print(",");
  Serial.print(gyro[1]);Serial.print(",");
  Serial.print(gyro[2]);Serial.println();
  /////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////
  Serial.print("#corrected gyro:");
  Serial.print(gyro_x);Serial.print(",");
  Serial.print(gyro_y);Serial.print(",");
  Serial.print(gyro_z);Serial.println();
  /////////////////////////////////////////////////////////
  // using quatanion differential to update
  //quatanion interation
  //////////////////////////////////////////////////////////
  Serial.print("#quatanion before:");
  Serial.print(qua[0]);Serial.print(",");
  Serial.print(qua[1]);Serial.print(",");
  Serial.print(qua[2]);Serial.print(",");
  Serial.print(qua[3]);Serial.println();
  //////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////
  Serial.print("#interation time:");
  Serial.print(G_Dt);Serial.println();
  /////////////////////////////////////////////////////////
  qua[0] += (-qua[1] * gyro_x - qua[2] * gyro_y - qua[3] * gyro_z) * G_Dt / 2.0f;
  qua[1] += (qua[0] * gyro_x + qua[2] * gyro_z - qua[3] * gyro_y) *  G_Dt / 2.0f;
  qua[2] += (qua[0] * gyro_y - qua[1] * gyro_z + qua[3] * gyro_x) *  G_Dt / 2.0f;
  qua[3] += (qua[0] * gyro_z + qua[1] * gyro_y - qua[2] * gyro_x) *  G_Dt / 2.0f;
  qua_norm(qua); 
  //////////////////////////////////////////////////////////
  Serial.print("#quatanion after:");
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









