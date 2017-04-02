//------------------------------------------------------------------------
// CurieNeurons SimpleScript2,
// Similar to simple script plus
// illustrates the difference between a recognition in KNN and RBF mode,
// and the use of multiple contexts.
//
// http://general-vision.com/documentation/TM_NeuroMem_Technology_Reference_Guide.pdf */
//
// Copyright 2016 General Vision Inc.
// -----------------------------------------------------------------------
//
// The patterns are arrays of length LEN composed of identical values VAL
// They basically represent horizontal lines with different heights. This representation
// is easy to comprehend the distance between the learned lines (stored in the memory
// of the committed neurons) and a new input line

#include <CurieNeuronsGeek.h>
CurieNeurons hNN;

int dist, cat, nsr;
int neuronSize, neuronsAvailable, neuronsCommitted;

#define LEN 4
byte pattern[LEN]; // values must range between 0-255. Upper byte is discarded by CM1K

#define K 3
int dists[K], cats[K], nids[K];

void setup() {
  Serial.begin(9600);
  while (!Serial);    // wait for the serial port to open

  Serial.println("Welcome to the Neurons of Curie");
  
  // Initialize the neurons
  hNN.Init();
  hNN.getNeuronsInfo( &neuronSize, &neuronsAvailable, &neuronsCommitted);
  Serial.print("Neuron size ="); Serial.println(neuronSize);
  Serial.print("Neurons available ="); Serial.println(neuronsAvailable);
  Serial.print("Neurons committed ="); Serial.println(neuronsCommitted);
  
  //Build knowledge by learning 3 patterns with each constant values (respectively 11, 15 and 20)
  Serial.print("\nLearning three patterns...");
  for (int i=0; i<LEN; i++) pattern[i]=11;
  hNN.Learn(pattern,LEN, 55);
  for (int i=0; i<LEN; i++) pattern[i]=15;
  hNN.Learn(pattern,LEN, 33);
  for (int i=0; i<LEN; i++) pattern[i]=20;
  neuronsCommitted=hNN.Learn(pattern,LEN, 100);
   
  displayNeurons();

  hNN.SetKNN(); 
  Serial.print("\n\nRecognizing pattern ");
  for (int i=0; i<LEN; i++) pattern[i]=12;     
  for (int i=0; i<LEN; i++) { Serial.print(pattern[i]); Serial.print(", ");}
  Serial.print("\nin mode KNN (K=3) ");
  int responseNbr=hNN.Classify(pattern, LEN, K, dists, cats, nids);
  for (int i=0; i<responseNbr; i++)
  {
      Serial.print("\nFiring neuron="); Serial.print(nids[i]);
      Serial.print(", Category="); Serial.print(cats[i]);
      Serial.print(", at Distance="); Serial.print(dists[i]);
  }

  hNN.SetRBF();
  Serial.print("\n\nRecognizing pattern ");
  for (int i=0; i<LEN; i++) pattern[i]=12;     
  for (int i=0; i<LEN; i++) { Serial.print(pattern[i]); Serial.print(", ");}
  Serial.print("\nin mode RBF (K up to 3) ");
  responseNbr=hNN.Classify(pattern, LEN, K, dists, cats, nids);
  for (int i=0; i<responseNbr; i++)
  {
      Serial.print("\nFiring neuron ID="); Serial.print(nids[i]);
      Serial.print(", Category="); Serial.print(cats[i]);
      Serial.print(", at Distance="); Serial.print(dists[i]);
  }
  
  // change the context to witness that a 2nd sub-network can be trained independantly
  // of the content of the 1st sub-network
  hNN.GCR(2);
  Serial.print("\n\nLearning a new pattern (13) under a different context=2");
  for (int i=0; i<LEN; i++) pattern[i]=13;
  hNN.Learn(pattern,LEN, 100);

  displayNeurons();
  Serial.print("\nNotice the addition of neuron4 with no impact on the influence fields of neuron1 and 2");
  
  hNN.GCR(1);
  Serial.print("\n\nLearning the same example (13) under context=1");
  for (int i=0; i<LEN; i++) pattern[i]=13;
  neuronsCommitted=hNN.Learn(pattern,LEN, 100);

  displayNeurons();
  Serial.print("\nNotice the addition of neuron5 and the shrinking of the influence fields of neuron1 and 2");

}

void loop()
{
  //Prompt User for input
  Serial.print("\n\nEdit a value between [1 and 99] to recognize a new pattern + Enter");
  while (Serial.available()==0)  { }
  int VAL=Serial.parseInt();
  
  for (int i=0; i<LEN; i++) pattern[i]=VAL;     
  Serial.print("\npattern=");
  for (int i=0; i<LEN; i++) { Serial.print(pattern[i]); Serial.print(", ");}

  int responseNbr=hNN.Classify(pattern, LEN, K, dists, cats, nids);
  for (int i=0; i<responseNbr; i++)
  {
      Serial.print("\nFiring neuron="); Serial.print(nids[i]);
      Serial.print(", Category="); Serial.print(cats[i]);
      Serial.print(", at Distance="); Serial.print(dists[i]);
  }
 }

  void displayNeurons()
  {
    int ncr, cat, aif;
    int ncount= hNN.NCOUNT();
    byte model[NEURONSIZE];
    Serial.print("\n\nDisplay the neurons, count="); Serial.print(ncount);
    for (int i=1; i<=ncount; i++)
    {  
        hNN.ReadNeuron(i, &ncr, model, &aif, &cat);
        Serial.print("\nneuron "); Serial.print(i); Serial.print("\tmodel=");
        for (int k=0; k<LEN; k++) { Serial.print(model[k]); Serial.print(", ");} 
        Serial.print("\tncr="); Serial.print(ncr);  
        Serial.print("\taif="); Serial.print(aif);     
        Serial.print("\tcat="); Serial.print(cat);
    }
  }
