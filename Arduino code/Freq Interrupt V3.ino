const int inpPin = 2;  //Only pins 2 and 3 serve as interrupt pins on arduino
const int pulsesPerRotation = 190; //Need a more accurate measurement
//Perhaps not necessary to get speed, can just go with pulses per second at lowest and highest speeds instead
const int denoisePeriod = 4; //Paramter, can be changed to whatever is best

/*To do: 
  Measure noise length in microseconds at lowest and highest speeds
  Measure pulses per second at lowest and highest speeds
  Measure pulses per rotation
*/

//Interrupt func variables, any variable altered by interrupt need to be volatile
volatile unsigned int pulseCount = 0; 
volatile int newPulseTime = 0; //This is just a ghetto bool variable
volatile int pulseBeginBool = 0;
volatile int pulseEndBool = 0;
volatile long pulseLength = 0;
volatile long pulseBegin = 0;
volatile long pulseEnd = 0;
volatile float sumFreq = 0;
volatile int prevState=0;
volatile int curState=0;
unsigned long lastPrintTime = 0; //Main loop variables
int printState = 0;
unsigned int prevPulseCount = 0;

const int BaudRate = 19200;

void setup() {
  // put your setup code here, to run once:
  pinMode(inpPin, INPUT);
  attachInterrupt(0, state_change, CHANGE);
  Serial.begin(BaudRate);
}

void loop() {

  //Prints pulses per second
  if (millis() - lastPrintTime > 1000) { 
    //Serial.print("Pulses per second: ");
    //Serial.println(pulseCount-prevPulseCount);
    lastPrintTime = millis();
    prevPulseCount=pulseCount;
  }
  /*
  if (digitalRead(inpPin)==HIGH){
    Serial.println(1);
  }
  else {Serial.println(0);}
  //Batch frequency readings from period, helps denoise. Stochastic frequency readings are in the interrupt function
  /*
  if (pulseCount%denoisePeriod==0 && printState==0){
    Serial.println(sumFreq/denoisePeriod);
    sumFreq=0;
    printState=1;
  } else if (pulseCount%denoisePeriod!=0 && printState==1){
    printState=0;
  } */
  
  //Printing number of pulses on command. Use for custom timing
  /*
  if (Serial.available()!=0){ //put any int in to print
    int readInt=Serial.parseInt();
    Serial.println(pulseCount-prevPulseCount);
    prevPulseCount=pulseCount;
  }
  */
}

void state_change() { //Function takes 8-70 microseconds, based on if statements
  //unsigned long startTime=micros();

  prevState=curState;
  curState=digitalRead(inpPin);

  if (digitalRead(inpPin) == HIGH) {
    if (pulseBeginBool==0 && pulseEndBool==0){ //State A
      pulseBeginBool=1;
      pulseBegin=micros();
    }
    else if (pulseBeginBool==1 && pulseEndBool==1){ //State C
      if (micros()-pulseEnd<300){
        pulseEndBool==0;
      }
      else{
        //newPulseTime==1;
        pulseLength=pulseBegin-pulseEnd;
        pulseCount++;
        pulseBegin=0;
        pulseEnd=0;
        Serial.println(pulseLength);
        pulseEndBool=0;
        pulseBeginBool=0;
      }
    }
    else{ //Default
      pulseEndBool=0;
      pulseBeginBool=0;      
    }
  } 
  else (digitalRead(inpPin)==LOW) {
    if (pulseBeginBool==1 && pulseEndBool==0){ //State B
      if(micros()-pulseBegin<300){
        pulseBeginBool==0;
      }
      else{
        pulseEndBool==1;
        pulseEnd=micros();
      }
    }
    else{ //Default
      pulseEndBool=0;
      pulseBeginBool=0;
    }
  }
  
  
  
  /*
  if (newPulseTime == 1) {
    pulseLength = pulseEnd - pulseBegin; 
    //Serial.println(pulseLength);
    if (pulseLength>800 && pulseLength<1000000)
      //float pulseFreqRealTime=1000000/(2*pulseLength);
      //Serial.println(pulseFreqRealTime); //Gives pulse frequency in real time from pulse period
      Serial.println(pulseLength); //For debugging
      pulseCount++;
      //sumFreq=sumFreq+pulseFreqRealTime;

      newPulseTime = 0;
    
  }*/
  //Serial.println(micros()-startTime);
}