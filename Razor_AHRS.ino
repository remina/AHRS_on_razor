/***************************************************************************************************************
* Razor AHRS Firmware 
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
*
*
*   * Allow optional use of EEPROM for storing and reading calibration values.
*   * Use self-test and temperature-compensation features of the sensors.
***************************************************************************************************************/

/*
  "9DOF Razor IMU" hardware versions: SEN-10125 and SEN-10736

  ATMega328@3.3V, 8MHz

  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10125
  HMC5883L : Magnetometer on SEN-10736
  ITG-3200 : Gyro

  Arduino IDE : Select board "Arduino Pro or Pro Mini (3.3v, 8Mhz) w/ATmega328"
*/

/*
  "9DOF Sensor Stick" hardware versions: SEN-10183, SEN-10321 and SEN-10724

  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10183 and SEN-10321
  HMC5883L : Magnetometer on SEN-10724
  ITG-3200 : Gyro
*/

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down.
    
  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up
  
  Transformation order: first yaw then pitch then roll.
*/

/*
  Serial commands that the firmware understands:
  
  "#o<params>" - Set OUTPUT mode and parameters. The available options are:
  
      // Streaming output
      "#o0" - DISABLE continuous streaming output. Also see #f below.
      "#o1" - ENABLE continuous streaming output.
      
      // Angles output
      "#ob" - Output angles in BINARY format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).

      // newly added commands
      "#oq" - Output quatanion in TEXT format (Output frames have form like "#QUA=xx.xx,-xx.xx,xx.xx,xx.xx",
              followed by carriage return and line feed [\r\n]).

      //change parameters: quatanion update alglorathim change P & I，correct GYRO_GAIN 
      "#xpxxx" - change proportion , proportion is xxx/10000

      "#xixxx" - change integral , integral is xxx/10000
	  
	  "#xgxxx" - change gyro gain ,gain is xxx/10000
      
      // Sensor calibration
      "#oc" - Go to CALIBRATION output mode.
      "#on" - When in calibration mode, go on to calibrate NEXT sensor.
      
      // Sensor data output
      "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
                NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards.
                In fact it's too much and an output frame rate of 50Hz can not be maintained. #osbb.
      "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
                One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).
      
      // Error message output        
      "#oe0" - Disable ERROR message output.
      "#oe1" - Enable ERROR message output.
    
    
  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only. Though #f only requests one reply, replies are still
         bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.
         
         
  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present and answering. The tracker will send
         "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
         x and y are two mandatory but arbitrary bytes that can be used to find out which request
         the answer belongs to.
          
          
  ("#C" and "#D" - Reserved for communication with optional Bluetooth module.)
  
  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.
  
  The status LED will be on if streaming output is enabled and off otherwise.
  
  Byte order of binary output is little-endian: least significant byte comes first.
*/



/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)


// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 9600

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 450  // in milliseconds
#define CALCULATE__DATA_INTERVAL 20

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes

// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float
#define OUTPUT__FORMAT_QUATANION 2 //outputs yaw/pitch/roll as quatanion

// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

// Bluetooth
// You can set this to true, if you have a Rovering Networks Bluetooth Module attached.
// The connect/disconnect message prefix of the module has to be set to "#".
// (Refer to manual, it can be set like this: SO,#)
// When using this, streaming output will only be enabled as long as we're connected. That way
// receiver and sender are synchronzed easily just by connecting/disconnecting.
// It is not necessary to set this! It just makes life easier when writing code for
// the receiving side. The Processing test sketch also works without setting this.
// NOTE: When using this, OUTPUT__STARTUP_STREAM_ON has no effect!
#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false


// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -255)
#define ACCEL_X_MAX ((float) 264)
#define ACCEL_Y_MIN ((float) -239)
#define ACCEL_Y_MAX ((float) 264)
#define ACCEL_Z_MIN ((float) -229)
#define ACCEL_Z_MAX ((float) 275)

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -306)
#define MAGN_X_MAX ((float) 466)
#define MAGN_Y_MIN ((float) -313)
#define MAGN_Y_MAX ((float) 452)
#define MAGN_Z_MIN ((float) -255)
#define MAGN_Z_MAX ((float) 256)

// Magnetometer (extended calibration mode)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {0, 0, 0};
//const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 22.69)
#define GYRO_AVERAGE_OFFSET_Y ((float) 19.72)
#define GYRO_AVERAGE_OFFSET_Z ((float) -1.09)

// Gain for gyroscope (ITG-3205)
float gyro_gain = 0.115; // Same gain on all axes

// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false
/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/










// Check if hardware version code is defined
#ifndef HW__VERSION_CODE
  // Generate compile error
  #error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.ino!
#endif

#include <Wire.h>

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// Stuff
#define STATUS_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi



// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];
float accel_store[3][9];// for filterring

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];
float magnetom_store[3][9];// for filterring

float gyro[3];
float gyro_average[3];
float ripe_gyro[3];
float gyro_store[3][9];// for filterring
int gyro_num_samples = 0;

// init 
float MAG_Heading;

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
unsigned long timestamp_out;
float G_Dt; // Integration time for algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

//*************************************NEWLY ADDED FOR AHRS ARGLORITHM**********************************************//
float qua[4] = {1,0,0,0};
float q0q1 = 0.0, q0q2 = 0.0, q0q3 = 0.0, q1q1 = 0.0, q1q2 = 0.0, 
      q1q3 = 0.0, q2q2 = 0.0, q2q3 = 0.0, q3q3 = 0.0;

//estimated dorection of gravity
float vx = 0.0, vy = 0.0, vz = 0.0;
//estimated dorection of mag
float wx = 0.0, wy = 0.0, wz = 0.0;

float ex = 0.0, ey = 0.0, ez = 0.0;
float integral_x = 0.0, integral_y = 0.0, integral_z = 0.0;

float two_kp = 0.06, two_ki = 0.0;

// for filterring
float accel_mean[3] = {0.0};
float gyro_mean[3] = {0.0};
float magnetom_mean[3] = {0.0};
unsigned int counter = 0;

void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor 9 times, record a time stamp and for 3 2-d arraies for filter
// Init alglorathim with an filtered orientation
// TODO re-init global vars?
void reset_sensor_fusion() 
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  // form filtering arraies
  for (; counter < 9; counter++)
  {
	  read_sensors();
	  // Apply sensor calibration
	  compensate_sensor_errors();
	  for(char i = 0; i < 3; i++)
	  {
		accel_store[i][counter] = accel[i];
		magnetom_store[i][counter] = magnetom[i];
		gyro_store[i][counter] = gyro[i]; 
	  }
  }
  // calculate mean
  for(char i = 0; i < 3; i++)
  {
	accel_mean[i] = mean(accel_store[i],9);
	gyro_mean[i] = mean(gyro_store[i],9);
	magnetom_mean[i] = mean(magnetom_store[i],9);
  }
  
  //**************************newly added normlize sensor output********************//
  norm(accel_mean, (accel_mean + 1), (accel_mean + 2));
  norm(magnetom_mean, (magnetom_mean + 1), (magnetom_mean + 2));
  
  timestamp = millis();
  timestamp_old = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = atan2(accel_mean[0], sqrt(accel_mean[1] * accel_mean[1] + accel_mean[2] * accel_mean[2]));
	
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel_mean, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = -atan2(temp2[1], temp2[2]);
  
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  
  Serial.print("###############################################");
  
  //////////////////////////////////////////////////////////
  Serial.print("#mag_mean:");
  Serial.print(magnetom_mean[0]);Serial.print(",");
  Serial.print(magnetom_mean[1]);Serial.print(",");
  Serial.print(magnetom_mean[2]);Serial.println();
  Serial.print("#gravity_mean:");
  Serial.print(accel_mean[0]);Serial.print(",");
  Serial.print(accel_mean[1]);Serial.print(",");
  Serial.print(accel_mean[2]);Serial.println();
  //////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////
  Serial.print("#init YPR:");
  Serial.print(TO_DEG(yaw));Serial.print(",");
  Serial.print(TO_DEG(pitch));Serial.print(",");
  Serial.print(TO_DEG(roll));Serial.println();
  //////////////////////////////////////////////////////////////
  
  // Init quatanion
  init_quatanion(qua, yaw, pitch, roll);
  //////////////////////////////////////////////////////////
  Serial.print("#init quatanion:");
  Serial.print(qua[0]);Serial.print(",");
  Serial.print(qua[1]);Serial.print(",");
  Serial.print(qua[2]);Serial.print(",");
  Serial.print(qua[3]);Serial.println();
  //////////////////////////////////////////////////////////////
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;

}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?only once!
  if (!reset_calibration_session_flag) return;
  
  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = 0.00f;
    magnetom_min[i] = magnetom_max[i] = 0.00f;
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
  
  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

// Blocks until another byte is available on serial port
char readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}

void setup()
{
  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);
  
  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  // Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
  turn_output_stream_off();
#else
  turn_output_stream_on();
#endif
}

          
// Main loop
void loop()
{
  ///////////////////////////////////////////////////////
  //Serial.print("#output:");
  //Serial.print(output_mode);Serial.print(",");
  //Serial.print(output_format);Serial.println();
  /////////////////////////////////////////////////////////
  // Read incoming control messages
  if (Serial.available() >= 2)
  {
    
	if (Serial.read() == '#') // Start of new control message
    {
      int command = Serial.read(); // Commands
      if (command == 'f') // request one output _f_rame
        output_single_on = true;
      //***********************************newly added**************************************************//
      else if (command == 'x')
      {
        char pi = readChar();
        float num;
        for(char k = 0; k <= 2; k++)
        {
            num += float(Serial.read() - '0');
            num *= 10.0f;
        }
        num /= 10000.0f;
        if (pi == 'p')
          two_kp = num;
        else if (pi == 'i')
          two_ki = num;
		else if (pi == 'g')
		  gyro_gain = num;
		else ;
      }
      else if (command == 's') // _s_ynch request
      {
        // Read ID
        byte id[2];
        id[0] = readChar();
        id[1] = readChar();
        
        // Reply with synch message
        Serial.print("#SYNCH");
        Serial.write(id, 2);
        Serial.println();
      }
      else if (command == 'o') // Set _o_utput mode
      {
        char output_param = readChar();
        if (output_param == 'n')  // Calibrate _n_ext sensor
        {
          curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 't') // Output angles as _t_ext
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_TEXT;
        }
        else if (output_param == 'b') // Output angles in _b_inary format
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_BINARY;
        }
        //*************************************newly added ******************************************//
        else if (output_param == 'q') //output angle in quatanion
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_QUATANION;
        }
        else if (output_param == 'c') // Go to _c_alibration mode
        {
          output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 's') // Output _s_ensor values
        {
          char values_param = readChar();
          char format_param = readChar();
          if (values_param == 'r')  // Output _r_aw sensor values
            output_mode = OUTPUT__MODE_SENSORS_RAW;
          else if (values_param == 'c')  // Output _c_alibrated sensor values
            output_mode = OUTPUT__MODE_SENSORS_CALIB;
          else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
            output_mode = OUTPUT__MODE_SENSORS_BOTH;

          if (format_param == 't') // Output values as _t_text
            output_format = OUTPUT__FORMAT_TEXT;
          else if (format_param == 'b') // Output values in _b_inary format
            output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == '0') // Disable continuous streaming output
        {
          turn_output_stream_off();
          reset_calibration_session_flag = true;
        }
        else if (output_param == '1') // Enable continuous streaming output
        {
          reset_calibration_session_flag = true;
          turn_output_stream_on();
        }
        else if (output_param == 'e') // _e_rror output settings
        {
          char error_param = readChar();
          if (error_param == '0') output_errors = false;
          else if (error_param == '1') output_errors = true;
          else if (error_param == 'c') // get error count
          {
            Serial.print("#AMG-ERR:");
            Serial.print(num_accel_errors); Serial.print(",");
            Serial.print(num_magn_errors); Serial.print(",");
            Serial.println(num_gyro_errors);
          }
        }
      }
#if OUTPUT__HAS_RN_BLUETOOTH == true
      // Read messages from bluetooth module
      // For this to work, the connect/disconnect message prefix of the module has to be set to "#".
      else if (command == 'C') // Bluetooth "#CONNECT" message (does the same as "#o1")
        turn_output_stream_on();
      else if (command == 'D') // Bluetooth "#DISCONNECT" message (does the same as "#o0")
        turn_output_stream_off();
#endif // OUTPUT__HAS_RN_BLUETOOTH == true
    }
    else
    { } // Skip character
  }

  // Time to read the sensors again?
  if((millis() - timestamp) >= CALCULATE__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
	timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();

    if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
    {
      check_reset_calibration_session();  // Check if this session needs a reset
    }
    else if (output_mode == OUTPUT__MODE_ANGLES)  // Output angles
    {
       //*********************newly added**************************************//
       // Serial.print("#raw accel:");
       // Serial.print(accel[0]);Serial.print(",");
       // Serial.print(accel[1]);Serial.print(",");
       // Serial.print(accel[2]);Serial.println();
       // Serial.print("#raw mag:");
       // Serial.print(magnetom[0]);Serial.print(",");
       // Serial.print(magnetom[1]);Serial.print(",");
       // Serial.print(magnetom[2]);Serial.println();
       // Serial.print("#raw gyro:");
       // Serial.print(gyro[0]);Serial.print(",");
       // Serial.print(gyro[1]);Serial.print(",");
       // Serial.print(gyro[2]);Serial.println();
       //**************************************************************************//
	   ///////////////////////////////////////////////////////
	   // Serial.print("#gyro_gain:");
	   // Serial.print(gyro_gain);Serial.println();
	   /////////////////////////////////////////////////////////
       // Apply sensor calibration & output mean
	   sensor_filter();
       //**************************newly added normlize sensor output********************//
       norm(accel_mean, (accel_mean + 1), (accel_mean + 2));
       norm(magnetom_mean, (magnetom_mean + 1), (magnetom_mean + 2));
    
       //**************************convert gyro to rad & apply a scale*************************//
	   ripe_gyro[0]=gyro_scaled_rad(gyro_mean[0]); //gyro x roll
	   ripe_gyro[1]=gyro_scaled_rad(gyro_mean[1]); //gyro y pitch
       ripe_gyro[2]=gyro_scaled_rad(gyro_mean[2]); //gyro z yaw
       // Run qua_update algorithm
       error_calaulate();
       quatanion_update();
     }
     else  // Output sensor values
     {       
     }
    
    output_single_on = false;
    
#if DEBUG__PRINT_LOOP_TIME == true
    Serial.print("loop time (ms) = ");
    Serial.println(millis() - timestamp);
#endif
  }
  if ((millis() - timestamp_out) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_out = millis();
	if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
	{
	   if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
	} 
	else if (output_mode == OUTPUT__MODE_ANGLES)
	{
	  if (output_stream_on || output_single_on) 
	  {
		if (output_format == OUTPUT__FORMAT_BINARY || output_format == OUTPUT__FORMAT_TEXT)
		{
		   ///////////////////////////////////////////////////////
		   // Serial.print("#PI parameters:");
           // Serial.print(two_kp);Serial.print(",");
           // Serial.print(two_ki);Serial.println();
           /////////////////////////////////////////////////////////
		   // convert to Euler_angles
		   qua_euler();
	       output_angles();
		}
		else if (output_format == OUTPUT__FORMAT_QUATANION)
		   output_quatanion();
		else
		{
		   // convert to Euler_angles
		   qua_euler();
		   output_angles();
		}
	  }  
	}
	else
	{
		if (output_stream_on || output_single_on) output_sensors();
	}
  }
#if DEBUG__PRINT_LOOP_TIME == true
  else
  {
    Serial.println("waiting...");
  }
#endif
}
