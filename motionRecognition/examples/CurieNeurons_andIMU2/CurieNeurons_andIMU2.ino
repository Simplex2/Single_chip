/* 
 *  ===============================================
  Example sketch using the Intel CurieIMU library and the General Vision CurieNeurons library 
  for Intel(R) Curie(TM) devices

  This script CurieNeurons_andIMU2 is very similar to CurieNeurons_andIMU except that it classifies
  motion using the redundancy between the acceleration and gyro signals. If you run both scripts,
  you should notice that CurieNeurons_andIMU2 produces many less false positive.
  
  Two simple feature vectors are extracted from the motion:
  Acceleration context (GCR=1), vector [ax'1, ay'1, az'1, ax'2, ay'2, az'2, ...ax'n, ay'n, az'n]
  Gyroscope context (GCR=2), vector [gx'1,gy'1, gz'1, gx'2, gy'2, gz'2, ...gx'n, gy'n, gz'n]
  Note that the values a' and g' are normalized and expand between the running min and max of the a and g signals.
  The program considers the motion as recognized positively if both contexts agree with its classification.

  After calibration is made,
  Use the serial monitor to edit the category of a motion, 
    (ex= 1 for vertical, 2 for horizontal, 0 for still or any other motion),
  Start moving the Curie in an expected direction,
  and when you press Enter the last feature vector is learned.
  
  Note that this "snapshot" approach is simplistic and you may have to teach several times
  a given motion so the neurons store models with different amplitudes, acceleration, etc.
  Ideally we want to learn consecutives vectors for a few seconds.
  
  ===============================================
*/
#include "CurieIMU.h"

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

#include <CurieNeuronsGeek.h>
CurieNeurons hNN;

#define sampleNbr 10  // number of samples to assemble a vector
int raw_vector[sampleNbr*3]; // vector accumulating sensor data
byte vector1[sampleNbr*3]; // vector holding the pattern context 1
byte vector2[sampleNbr*3]; // vector holding the pattern context 2
int cat1, cat2;
int catL=0, dist, nid, nsr, ncount;
int sampleId=0, prevcat=0; // loop variables

// Variables to normalize the vector data
int mina=0xFFFF, maxa=0, ming=0xFFFF, maxg=0;
int da, dg;

void setup() {
  Serial.begin(250000); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();

  // verify connection
  Serial.println("Testing device connections...");
  if (CurieIMU.begin()) {
    Serial.println("CurieIMU connection successful");
  } else {
    Serial.println("CurieIMU connection failed");
  }

  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //CurieIMU.setAccelerometerOffset(X_AXIS,128);
    //CurieIMU.setAccelerometerOffset(Y_AXIS,-4);
    //CurieIMU.setAccelerometerOffset(Z_AXIS,127);
    //CurieIMU.setGyroOffset(X_AXIS,129);
    //CurieIMU.setGyroOffset(Y_AXIS,-1);
    //CurieIMU.setGyroOffset(Z_AXIS, 254);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  }
    
  // Initialize the neurons and set a conservative Max Influence Field
  hNN.Init();  
  hNN.Forget(1000); //set a conservative  Max Influence Field prior to learning
  
  Serial.print("\n\nEntering loop...");
  Serial.print("\nMove the module vertically or horizontally...");
  Serial.print("\ntype 1 + Enter if vertical motion");
  Serial.print("\ntype 2 + Enter if horizontal motion");
  Serial.print("\ntype 0 + Enter for any other motion");
}

void loop() 
{
    getVector1(); // acceleration vector
    getVector2(); // gyro vector
       
    // Learn if push button depressed and report if a new neuron is committed
    if (Serial.available() == 2) 
    {
      catL = Serial.read();
      char inChar = (char)Serial.read();
      if (inChar == '\n')
      {
        catL = catL - 48;
        hNN.GCR(1);
        ncount=hNN.Learn(vector1, sampleNbr*3, catL);
        hNN.GCR(2);
        ncount=hNN.Learn(vector2, sampleNbr*3, catL);
        Serial.print("\nLearning motion category "); Serial.print(catL);
        Serial.print("\tNeurons="); Serial.print(ncount);
        //Serial.print("\nVector1 = ");
        //for (int i=0; i<3; i++) {Serial.print(vector1[i]);Serial.print("\t");}
        //Serial.print("\nVector2 = ");
        //for (int i=0; i<3; i++) {Serial.print(vector2[i]);Serial.print("\t");}       
      }
    }
    else
    {
      // Recognize
      hNN.GCR(1);
      hNN.Classify(vector1, sampleNbr*3,&dist, &cat1, &nid);
      hNN.GCR(2);
      hNN.Classify(vector2, sampleNbr*3,&dist, &cat2, &nid);
      if (cat1==cat2)
      {
        if (cat1!=prevcat)
        {
          if (cat1!=0x7FFF)
          {
            Serial.print("\nMotion category #"); Serial.print(cat1);
          }
          else Serial.print("\nMotion unknown");
        }      
        prevcat=cat1;
      }   
    }  
}

void getVector1()
{
  // the reset of the min and max values is optional depending if you want to
  // use a running min and max from the launch of the script or not
  mina=0xFFFF, maxa=0, da;
  
  for (int sampleId=0; sampleId<sampleNbr; sampleId++)
  {
    //Build the vector over sampleNbr and broadcast to the neurons
    CurieIMU.readAccelerometer(ax, ay, az);
    
    // update the running min/max for the a signals
    if (ax>maxa) maxa=ax; else if (ax<mina) mina=ax;
    if (ay>maxa) maxa=ay; else if (ay<mina) mina=ay;
    if (az>maxa) maxa=az; else if (az<mina) mina=az;    
    da= maxa-mina;
    
    // accumulate the sensor data
    raw_vector[sampleId*3]= ax;
    raw_vector[(sampleId*3)+1]= ay;
    raw_vector[(sampleId*3)+2]= az;
  }
  
  // normalize vector
  for(int sampleId=0; sampleId < sampleNbr; sampleId++)
  {
    for(int i=0; i<3; i++)
    {
      vector1[(sampleId*3)+i]  = (((raw_vector[(sampleId*3)+i] - mina) * 255)/da) & 0x00FF;
    }
  }
}

void getVector2()
{
  // the reset of the min and max values is optional depending if you want to
  // use a running min and max from the launch of the script or not
  ming=0xFFFF, maxg=0, dg;
  
  for (int sampleId=0; sampleId<sampleNbr; sampleId++)
  {
    //Build the vector over sampleNbr and broadcast to the neurons
    CurieIMU.readGyro(gx, gy, gz);
    
    // update the running min/max for the g signals
    if (gx>maxg) maxg=gx; else if (gx<ming) ming=gx;
    if (gy>maxg) maxg=gy; else if (gy<ming) ming=gy;
    if (gz>maxg) maxg=gz; else if (gz<ming) ming=gz;   
    dg= maxg-ming;

    // accumulate the sensor data
    raw_vector[(sampleId*3)]= gx;
    raw_vector[(sampleId*3)+1]= gy;
    raw_vector[(sampleId*3)+2]= gz;
  }
  
  // normalize vector
  for(int sampleId=0; sampleId < sampleNbr; sampleId++)
  {
    for(int i=0; i<3; i++)
    {
      vector2[(sampleId*3)+i]  = (((raw_vector[(sampleId*3)+i] - ming) * 255)/dg) & 0x00FF;
    }
  }
}
