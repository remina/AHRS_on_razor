/* This file is part of the Razor AHRS Firmware */

// Output angles: yaw, pitch, roll

void output_angles()
{
  if (output_format == OUTPUT__FORMAT_BINARY)
  {
    float ypr[3];  
    ypr[0] = TO_DEG(yaw);
    ypr[1] = TO_DEG(pitch);
    ypr[2] = TO_DEG(roll);
    Serial.write((byte*) ypr, 12);  // No new-line
  }
  else if (output_format == OUTPUT__FORMAT_TEXT)
  {
    Serial.print("#YPR=");
    Serial.print(TO_DEG(yaw)); Serial.print(",");
    Serial.print(TO_DEG(pitch)); Serial.print(",");
    Serial.print(TO_DEG(roll)); Serial.println();
  }
}

void output_angles_onboard()
{
    int ypr[3];  
    ypr[0] = TO_DEG(yaw) * 100;
    ypr[1] = TO_DEG(pitch) * 100;
    ypr[2] = TO_DEG(roll) * 100;
	byte sum = 341;
	String temp;
	
    Serial.print(85,HEX);
    Serial.print(86,HEX);
	for(int i = 0; i < 1; i++)
	{
		temp = String(ypr[i],HEX);
		temp = temp.substring(0,4);
		Serial.print(temp);
		if(temp[0] == 'a' || temp[0] == 'b' || temp[0] == 'c' || temp[0] == 'd' || temp[0] == 'e' || temp[0] == 'f')
		{
			sum += (temp[0] - 'a' + 10) * 16;
		}
		else{
			sum += (temp[0] - '0') * 16;
		}
		if(temp[1] == 'a' || temp[1] == 'b' || temp[1] == 'c' || temp[1] == 'd' || temp[1] == 'e' || temp[1] == 'f')
		{
			sum += temp[1] - 'a' + 10;
		}
		else{
			sum += temp[1] - '0';
		}
		if(temp[2] == 'a' || temp[2] == 'b' || temp[2] == 'c' || temp[2] == 'd' || temp[2] == 'e' || temp[2] == 'f')
		{
			sum += (temp[2] - 'a' + 10) * 16;
		}
		else{
			sum += (temp[2] - '0') * 16;
		}
		if(temp[3] == 'a' || temp[3] == 'b' || temp[3] == 'c' || temp[3] == 'd' || temp[3] == 'e' || temp[3] == 'f')
		{
			sum += temp[3] - 'a' + 10;
		}
		else{
			sum += temp[3] - '0';
		}
        // for(int i = 0; i < 4; i++)
        // {
			// if(temp[i] == 'a' || temp[i] == 'b' || temp[i] == 'c' || temp[i] == 'd' || temp[i] == 'e' || temp[i] == 'f')
			// {
				// if(i == 3 || i == 1)
				// {
					// sum += (temp[i] - 'a' + 10) * 16;
				// }
				// else
					// sum += temp[i] - 'a' + 10;
			// }
			// else
			// {
				// if(i == 3 || i == 1)
				// {
					// sum += (temp[i] - '0') * 16;
				// }
				// else
					// sum += temp[i] - '0';
			// }
				
        // }
		
	}
    Serial.print(85,HEX);
	Serial.print(85,HEX);
	temp = String(sum,HEX);
	temp = temp.substring(0,2);
	Serial.print(temp);
    Serial.println();
}

void output_calibration(int calibration_sensor)
{
  if (calibration_sensor == 0)  // Accelerometer
  {
    // Output MIN/MAX values
    Serial.print("accel x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
      if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
      Serial.print(accel_min[i]);
      Serial.print("/");
      Serial.print(accel_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 1)  // Magnetometer
  {
    // Output MIN/MAX values
    Serial.print("magn x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
      if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
      Serial.print(magnetom_min[i]);
      Serial.print("/");
      Serial.print(magnetom_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 2)  // Gyroscope
  {
    // Average gyro values
    for (int i = 0; i < 3; i++)
      gyro_average[i] += gyro[i];
    gyro_num_samples++;
      
    // Output current and averaged gyroscope values
    Serial.print("gyro x,y,z (current/average) = ");
    for (int i = 0; i < 3; i++) {
      Serial.print(gyro[i]);
      Serial.print("/");
      Serial.print(gyro_average[i] / (float) gyro_num_samples);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
}

void output_sensors_text(char raw_or_calibrated)
{
  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(accel[0]); Serial.print(",");
  Serial.print(accel[1]); Serial.print(",");
  Serial.print(accel[2]); Serial.println();

  Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(magnetom[0]); Serial.print(",");
  Serial.print(magnetom[1]); Serial.print(",");
  Serial.print(magnetom[2]); Serial.println();

  Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(gyro[0]); Serial.print(",");
  Serial.print(gyro[1]); Serial.print(",");
  Serial.print(gyro[2]); Serial.println();
}

void output_sensors_binary()
{
  Serial.write((byte*) accel, 12);
  Serial.write((byte*) magnetom, 12);
  Serial.write((byte*) gyro, 12);
}

void output_sensors()
{
  if (output_mode == OUTPUT__MODE_SENSORS_RAW)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('R');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
  {
    // Apply sensor calibration
    compensate_sensor_errors();
    
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('C');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_BOTH)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
    {
      output_sensors_binary();
      compensate_sensor_errors();
      output_sensors_binary();
    }
    else if (output_format == OUTPUT__FORMAT_TEXT)
    {
      output_sensors_text('R');
      compensate_sensor_errors();
      output_sensors_text('C');
    }
  }
}

// Output angles in quatanion
void output_quatanion()
{
  Serial.print("#quatanion=");
  Serial.print(qua[0]); Serial.print(",");
  Serial.print(qua[1]); Serial.print(",");
  Serial.print(qua[2]); Serial.print(",");
  Serial.print(qua[3]); Serial.println();
}
