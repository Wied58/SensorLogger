 
/*
  Initial version from Jeff Wiedemann's code of May 2014.
  Written for Teensy 3.0.
  See file "pressure_and_accel_and_mag_4_w_gps_4.ino.ref".
  
  Modified to use a separate Teensy version of the HMC5883L library
  incorporating minor changes as used by Jeff.
*/


#include <Wire.h>
#include <math.h>
#include <ADXL345.h>
#include <HMC5883L_T.h>

ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
#define pressure 0x76 //0x77

#define mag 0x1E //0011110b, I2C 7bit address of HMC5883

uint32_t D1 = 0;
uint32_t D2 = 0;
float dT = 0;
int32_t TEMP = 0;
float OFF = 0; 
float SENS = 0; 
float P = 0;
float T2  = 0;
float OFF2  = 0;
float SENS2 = 0;
uint16_t C[7];
int count = 0;

float Temperature;
float Pressure;

int Ax,Ay,Az;  
double Axg, Ayg, Azg;

int inputpin0 = 14;
int inputpin1 = 15;
int inputpin2 = 16;
int inputpin3 = 17;

int input0val = 0;
int input1val = 0;
int input2val = 0;
int input3val = 0;


// Store our compass as a variable.
HMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;

 int line_count=1;


void setup() {
  


// Disable internal pullups, 10Kohms are on the breakout
 PORTC |= (1 << 4);
 PORTC |= (1 << 5);

  Wire.begin();
  Serial2.begin(57600); //9600 changed 'cos of timing?
  delay(1000); //Give the open log a second to get in gear. 
  initial(pressure);
  
  adxl.powerOn();
  
//Cliped from H5883L example
  
Serial2.println("Constructing new HMC5883L");
  compass = HMC5883L(); // Construct a new HMC5883 compass.
    
  Serial2.println("Setting scale to +/- 1.3 Ga");
  error = compass.SetScale(1.3); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    Serial2.println(compass.GetErrorText(error));
  
  Serial2.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial2.println(compass.GetErrorText(error));

Serial1.begin(57600);
Serial1.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
   for (int noodle = 1; noodle < 10; noodle++);
   
  pinMode(inputpin0, INPUT);
  pinMode(inputpin1, INPUT);
  pinMode(inputpin2, INPUT);
  pinMode(inputpin3, INPUT);

}

void loop()
{
  if (Serial1.available()) { //gps stuff 1
//    Serial2.write(Serial1.read());
 char gps_byte =Serial1.read();
 Serial2.print(gps_byte);
 if (gps_byte == 10){ //gps stuff 2
      line_count++;
    //  Serial2.println(line_count);   
   if (line_count == 2){ //gps stuff 3
   // Serial2.println(line_count);

count = count + 1;

/*******************************************************************************/
/**************              ACCELEROMETER                **********************/
/*******************************************************************************/
  
 /* adxl.readAccel(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z */

 adxl.setRangeSetting(16);
  
 adxl.readAccel(&Ax, &Ay, &Az); //read the accelerometer values and store them in variables  x,y,z
 
  
  


  
 Axg = (int16_t)Ax * 0.0313;
 Ayg = (int16_t)Ay * 0.0313;
 Azg = (int16_t)Az * 0.0313;
 

 
/*******************************************************************************/
/**************                PRESSURE                   **********************/
/*******************************************************************************/
  
  D1 = getVal(pressure, 0x48); // Pressure raw
  D2 = getVal(pressure, 0x58);// Temperature raw

  dT   = (float)D2 - ((uint32_t)C[5] * 256);
  OFF  = ((float)C[2] * 131072) + ((dT * C[4]) / 64);
  SENS = ((float)C[1] * 65536) + (dT * C[3] / 128);

  TEMP = (int64_t)dT * (int64_t)C[6] / 8388608 + 2000;
  
 if(TEMP < 2000) // if temperature lower than 20 Celsius 
  {
    
    T2=pow(dT,2)/2147483648;
    OFF2=61*pow((TEMP-2000),2)/16;
    SENS2=2*pow((TEMP-2000),2);
    
   if(TEMP < -1500) // if temperature lower than -15 Celsius 
    {
     OFF2=OFF2+15*pow((TEMP+1500),2);
     SENS2=SENS2+8*pow((TEMP+1500),2);
    }

  

 
    TEMP = TEMP - T2;
    OFF = OFF - OFF2; 
    SENS = SENS - SENS2;
    
    
  }
  
  

  Temperature = (float)TEMP / 100; 
  
  P  = (D1 * SENS / 2097152 - OFF) / 32768;
  
 

  Pressure = (float)P / 100;
  
/*******************************************************************************/
/**************                 SWITCHES                  **********************/
/*******************************************************************************/

input0val = digitalRead(inputpin0);
input1val = digitalRead(inputpin1);
input2val = digitalRead(inputpin2);
input3val = digitalRead(inputpin3);
  
/*******************************************************************************/
/**************                 MAGNETOMETER              **********************/
/*******************************************************************************/
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  
  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0457;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Output the data via the serial port.
  Output(raw, scaled, heading, headingDegrees);

  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L).
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
  // delay(66);
 
 line_count=0;
   } //gps stuff 3?
 
  } //gps stuff 3
 
  } // gps stuff 2
  
} //gps stuff 1



// Output the data down the serial port.
void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
{
   
/*******************************************************************************/
/**************                  OUTPUT                   **********************/
/*******************************************************************************/
   Serial2.print("$FHMAG");
   Serial2.print(",");
   Serial2.print(count);
   Serial2.print(",");
   Serial2.print(headingDegrees);
   Serial2.print(",");
   Serial2.print(heading);
   Serial2.print(",");
   Serial2.print(raw.XAxis);
   Serial2.print(",");   
   Serial2.print(raw.YAxis);
   Serial2.print(",");   
   Serial2.print(raw.ZAxis);
   Serial2.print(",");
   Serial2.print(scaled.XAxis);
   Serial2.print(",");
   Serial2.print(scaled.YAxis);
   Serial2.print(","); 
   Serial2.println(scaled.ZAxis);

  


  Serial2.print("$FHPRS");
  Serial2.print(",");
  Serial2.print(count);
  Serial2.print(",");
  Serial2.print(Temperature);
  Serial2.print(",");
  Serial2.print(Pressure);
  Serial2.print(",");
  Serial2.print(D2);
  Serial2.print(",");
  Serial2.print(D1);
  Serial2.print(",");
  Serial2.print(dT);
  Serial2.print(",");
  Serial2.print(SENS);
  Serial2.print(",");
  Serial2.print(OFF/100);
  Serial2.print(",");
  Serial2.print(T2);
  Serial2.print(",");
  Serial2.print(SENS2);
  Serial2.print(",");
  Serial2.println(OFF2/100);
  
  
  
  Serial2.print("$FHACC");
  Serial2.print(",");
  Serial2.print(count);
  Serial2.print(",");
  Serial2.print((float)Axg,2);
  Serial2.print(",");
  Serial2.print((float)Ayg,2);
  Serial2.print(",");
  Serial2.print((float)Azg,2);
  Serial2.print(",");
  Serial2.print(Ax);
  Serial2.print(",");
  Serial2.print(Ay);
  Serial2.print(",");
  Serial2.println(Az);
   
  
  Serial2.print("$FHSWP");
  Serial2.print(",");
  Serial2.print(input0val);
  Serial2.print(",");
  Serial2.print(input1val);
  Serial2.print(",");
  Serial2.print(input2val);
  Serial2.print(",");
  Serial2.println(input3val);
  Serial2.println(); 
  
  
  
  
  /* RESET THE CORECTION FACTORS */
  
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;


/*  delay(1000); */

} /* END OF ARDUINO LOOP */

long getVal(int address, byte code)
{
  unsigned long ret = 0;
  Wire.beginTransmission(address);
  Wire.write(code);
  Wire.endTransmission();
  delay(10);
  // start read sequence
  Wire.beginTransmission(address);
  Wire.write((byte) 0x00);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom(address, (int)3);
  if (Wire.available() >= 3)
  {
    ret = Wire.read() * (unsigned long)65536 + Wire.read() * (unsigned long)256 + Wire.read();
  }
  else {
    ret = -1;
  }
  Wire.endTransmission();
  return ret;
}

void initial(uint8_t address)
{

  Serial2.println();
  Serial2.println("PRESSURE SENSOR PROM COEFFICIENTS");

  Wire.beginTransmission(address);
  Wire.write(0x1E); // reset
  Wire.endTransmission();
  delay(10);


  for (int i=0; i<6  ; i++) {

    Wire.beginTransmission(address);
    Wire.write(0xA2 + (i * 2));
    Wire.endTransmission();

    Wire.beginTransmission(address);
    Wire.requestFrom(address, (uint8_t) 6);
    delay(1);
    if(Wire.available())
    {
       C[i+1] = Wire.read() << 8 | Wire.read();
    }
    else {
      Serial2.println("Error reading PROM 1"); // error reading the PROM or communicating with the device
    }
    Serial2.println(C[i+1]);
  }
  Serial2.println();
}
