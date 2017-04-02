/*
 Name:		CurieNeurons_andIMU2.0.ino
 Created:	7/21/2016 11:08:34 AM
 Author:	HansYang
*/


#include <CurieIMU.h>
#include <BMI160.h>
#include <CurieNeurons.h>
#include <stdlib.h>
#define DEBUG1
#define ACC_FEATURE_ONLY
#define MANUAL_SCALING

#ifdef ACC_FEATURE_ONLY
#define SAMPLENBR 3  
#else
#define SAMPLENBR 6
#endif // ACC_FEATURE_ONLY

#define MAX_SAMPLING_TIME 3000//ms
#define SAMPLING_INTERVAL 5//ms
#define MAX_VECTOR_LEN 600//MAX_SAMPLING_TIME/SAMPLING_INTERVAL

#define DOWNSAMPLER_SEGS 10

//ADkeyboard
int adc_key_val[5] = { 810,840,870,910,980 };
#define NUM_KEYS 5
int adc_key_in;
int key = -1;

//CurieNeurons
CurieNeurons hNN;
int cat, prevcat;
int dist, nid;

//IMU data
int16_t ax, ay, az;
int16_t gx, gy, gz;

int minax, minay, minaz, maxax, maxay, maxaz;
int mingx, mingy, mingz, maxgx, maxgy, maxgz;
int Sax, Say, Saz, Sgx, Sgy, Sgz;

//Others
uint64_t timer;
short vector_len;
byte vector[MAX_VECTOR_LEN*SAMPLENBR];
byte pattern[SAMPLENBR * 17];
byte learnflag;

void feature_extraction(byte* src_vector, int src_len, byte *dst_vector)
{
	//��׼��, ƽ���������, ratio, seg_means
	int seg_len = src_len / (DOWNSAMPLER_SEGS * SAMPLENBR);
	double res[17 * SAMPLENBR];
	for (int icol = 0; icol < SAMPLENBR; icol++)
	{
		int sum = 0;
		for (int i = 0; i < src_len; i++)
		{
			sum += src_vector[i*SAMPLENBR + icol];
		}
		//mean
		int aver = sum / src_len;
		int e = 0;
		int mad = 0;
		//int rms = 0;
		//byte max = src_vector[icol];
		//byte min = src_vector[icol];
		int ratiocount[5] = { 0 };
		for (int i = 0; i < src_len; i++)
		{
			e += (src_vector[i*SAMPLENBR + icol] - aver)*(src_vector[i*SAMPLENBR + icol] - aver);
			mad += abs(src_vector[i*SAMPLENBR + icol] - aver);
			//rms += src_vector[i*SAMPLENBR + icol] * src_vector[i*SAMPLENBR + icol];
			//if (src_vector[i*SAMPLENBR + icol] > max)
			//	max = src_vector[i*SAMPLENBR + icol];
			//if (src_vector[i*SAMPLENBR + icol] < min)
			//	min = src_vector[i*SAMPLENBR + icol];
			if (src_vector[i*SAMPLENBR + icol] <= 200)
			{
				if (src_vector[i*SAMPLENBR + icol] <= 150)
				{
					if (src_vector[i*SAMPLENBR + icol] <= 100)
					{
						if (src_vector[i*SAMPLENBR + icol] <= 50)
						{
							ratiocount[4]++;
						}
						else
							ratiocount[3]++;
					}
					else
						ratiocount[2]++;
				}
				else
					ratiocount[1]++;
			}
			else
				ratiocount[0]++;
		}
		//standard deviation
		res[icol] = sqrt(e / (src_len - 1)) * 2;
		//MAD
		res[icol + 1 * SAMPLENBR] = mad / src_len * 2;
		//RMS
		//res[icol + 3 * SAMPLENBR] = sqrt(rms / src_len);
		for (int i = 0; i < 5; i++)
		{
			res[icol * 5 + 6 + i] = 0;//ratiocount[i];// * 255 / src_len;
		}

		for (int iseg = 0; iseg < DOWNSAMPLER_SEGS; iseg++)
		{
			int sum = 0;
			for (int ismp = 0; ismp < seg_len; ismp++)
			{
				int offset = icol + (iseg * seg_len + ismp)*SAMPLENBR;
				sum += (int)src_vector[offset];
				src_vector[offset] = 0;
			}
			res[20 + icol * DOWNSAMPLER_SEGS + iseg] = sum / seg_len;
		}
	}

	for (int i = 0; i < SAMPLENBR * 17; i++)
	{
		pattern[i] = (int)(res[i] + 0.5) * 0x00FF;
	}

#ifdef DEBUG1
	Serial.print("\n");
	for (int i = 0; i < SAMPLENBR*17; i++)
	{
		Serial.print(pattern[i]);
		Serial.print("\t");
	}
	//Serial.print("\n");
#endif // DEBUG1

}

int get_key(unsigned int input)	//0 1 2 3 4 5
{
	int k;
	for (k = 0; k < NUM_KEYS; k++)
	{
		if (input < adc_key_val[k])
		{
			return k + 1;
		}
	}
	if (k >= NUM_KEYS)
		k = -1;  // No valid key pressed
	return k + 1;
}

void read_data_once()
{
	CurieIMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	/*int R = sqrt(ax*ax + ay*ay + az*az);

	double tX = acos(ax / R);
	double tY = acos(ay / R);
	double tZ = acos(az / R);

	ax = ax - 8192 * cos(tX);
	ay = ay - 8192 * cos(tY);
	az = ax - 8192 * cos(tZ);*/
	
	vector[vector_len*SAMPLENBR + 0] = (byte)(((ax - minax) * 255 / Sax) & 0x00FF);
	vector[vector_len*SAMPLENBR + 1] = (byte)(((ay - minay) * 255 / Say) & 0x00FF);
	vector[vector_len*SAMPLENBR + 2] = (byte)(((az - minaz) * 255 / Saz) & 0x00FF);
#ifndef ACC_FEATURE_ONLY
	vector[vector_len*SAMPLENBR + 3] = (byte)(((gx - mingx) * 255 / Sgx) & 0x00FF);
	vector[vector_len*SAMPLENBR + 4] = (byte)(((gy - mingy) * 255 / Sgy) & 0x00FF);
	vector[vector_len*SAMPLENBR + 5] = (byte)(((gz - mingz) * 255 / Sgz) & 0x00FF);
#endif // !ACC_FEATURE_ONLY		

	/*vector[vector_len*SAMPLENBR + 0] = ax;
	vector[vector_len*SAMPLENBR + 1] = ay;
	vector[vector_len*SAMPLENBR + 2] = az;
#ifndef ACC_FEATURE_ONLY
	vector[vector_len*SAMPLENBR + 3] = gx;
	vector[vector_len*SAMPLENBR + 4] = gy;
	vector[vector_len*SAMPLENBR + 5] = gz;
#endif // !ACC_FEATURE_ONLY		*/

	if (vector_len < (MAX_VECTOR_LEN - 2)*SAMPLENBR ) {
		vector_len++;
		Serial.print("*");
	}
	else
	{
		Serial.print("O");
	}
}

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	while (!Serial);

	Serial.print("CurieNeurons initializing...");
	hNN.Init();
	hNN.Forget();
	Serial.println("Done");

	Serial.print("CurieIMU initializing...");
	CurieIMU.begin();
	Serial.println("Done");

	Serial.println("Initializing IMU device...");
	CurieIMU.begin();

	// use the code below to calibrate accel/gyro offset values
	Serial.println("Internal sensor offsets BEFORE calibration...");
	Serial.print(CurieIMU.getXAccelOffset());
	Serial.print("\t"); // -76
	Serial.print(CurieIMU.getYAccelOffset());
	Serial.print("\t"); // -235
	Serial.print(CurieIMU.getZAccelOffset());
	Serial.print("\t"); // 168
	Serial.print(CurieIMU.getXGyroOffset());
	Serial.print("\t"); // 0
	Serial.print(CurieIMU.getYGyroOffset());
	Serial.print("\t"); // 0
	Serial.println(CurieIMU.getZGyroOffset());

	Serial.println("About to calibrate. Make sure your board is stable and upright");
	delay(2000);

	// The board must be resting in a horizontal position for 
	// the following calibration procedure to work correctly!
	Serial.print("Starting Gyroscope calibration...");
	CurieIMU.autoCalibrateGyroOffset();
	Serial.println(" Done");
	Serial.print("Starting Acceleration calibration...");
	CurieIMU.autoCalibrateXAccelOffset(0);
	CurieIMU.autoCalibrateYAccelOffset(0);
	CurieIMU.autoCalibrateZAccelOffset(1);
	Serial.println(" Done");

	Serial.println("Internal sensor offsets AFTER calibration...");
	Serial.print(CurieIMU.getXAccelOffset());
	Serial.print("\t"); // -76
	Serial.print(CurieIMU.getYAccelOffset());
	Serial.print("\t"); // -2359
	Serial.print(CurieIMU.getZAccelOffset());
	Serial.print("\t"); // 1688
	Serial.print(CurieIMU.getXGyroOffset());
	Serial.print("\t"); // 0
	Serial.print(CurieIMU.getYGyroOffset());
	Serial.print("\t"); // 0
	Serial.println(CurieIMU.getZGyroOffset());

	Serial.println("Enabling Gyroscope/Acceleration offset compensation");
	CurieIMU.setGyroOffsetEnabled(true);
	CurieIMU.setAccelOffsetEnabled(true);

	CurieIMU.setAccelerometerRange(4);
	//hNN.GCR(128);			//LSUP distance
	//hNN.MAXIF(600);

#ifdef MANUAL_SCALING

	minax = minay = minaz = maxax = maxay = maxaz = -32768;
	mingx = mingy = mingz = maxgx = maxgy = maxgz = 32767;
	Sax = Say = Saz = Sgx = Sgy = Sgz = 65535;

#else
	Serial.println("\nAssessing the amplitude of motions...Press any key to continue");
	adc_key_in = analogRead(5);
	while (adc_key_in > 980)
	{
		adc_key_in = analogRead(0);
		CurieIMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		if (ax>maxax) maxax = ax; if (ax<minax) minax = ax;
		if (ay>maxay) maxay = ay; if (ay<minay) minay = ay;
		if (az>maxaz) maxaz = az; if (az<minaz) minaz = az;
		if (gx>maxgx) maxgx = gx; if (gx<mingx) mingx = gx;
		if (gy>maxgy) maxgy = gy; if (gy<mingy) mingy = gy;
		if (gz>maxgz) maxgz = gz; if (gz<mingz) mingz = gz;
	}
	Sax = maxax - minax;
	Say = maxay - minay;
	Saz = maxaz - minaz;
	Sgx = maxgx - mingx;
	Sgy = maxgy - mingy;
	Sgz = maxgz - mingz;
	Serial.print("\nSa :\t"); Serial.print(Sax); Serial.print(","); Serial.print(Say); Serial.print(","); Serial.print(Saz);
	Serial.print("\nSg :\t"); Serial.print(Sgx); Serial.print(","); Serial.print(Sgy); Serial.print(","); Serial.println(Sgz);
#endif

	timer = 0;
	prevcat = 0x7FFF;
	learnflag = 0;

	Serial.print("The program will begin in 2 seconds...\n");
	delay(2000);
	Serial.println("Start.");


}

// the loop function runs over and over again until power down or reset
void loop() {

	timer = millis();
	adc_key_in = analogRead(5);
	key = get_key(adc_key_in);

	if (key!=0)
	{
		learnflag = key;
		read_data_once();
	}
	else if (learnflag != 0)
	{
		read_data_once();
		feature_extraction(vector, vector_len, pattern);
		//downsampler_means(vector, vector_len, pattern, SAMPLENBR*DOWNSAMPLER_SEGS);
#ifdef DEBUG
		Serial.print("\n");
		for (int i = 0; i < SAMPLENBR*DOWNSAMPLER_SEGS; i++)
		{
			Serial.print(pattern[i]);
			Serial.print("\t");
		}
		Serial.print("\n");
#endif //DEBUG
		if (learnflag != 5)
		{
			hNN.Learn(pattern, SAMPLENBR * 17, learnflag);
			Serial.print("\nLearned motion #");
			Serial.print(learnflag);
			Serial.println("once, please repeat.");
		}
		else
		{
			int res = hNN.Classify(pattern, SAMPLENBR * 17, &dist, &cat, &nid);
			cat &= 0x00FF;
			if (res != 0)
			{
				Serial.print("\nMotion Detected. #"); Serial.println(cat);
			}
			else
			{
				Serial.println("\nMotion Unknow.");
			}
		}
		learnflag = 0;
		vector_len = 0;
	}
	while ((millis() - timer) < SAMPLING_INTERVAL);
}
