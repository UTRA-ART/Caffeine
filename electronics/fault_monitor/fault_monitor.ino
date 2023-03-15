// pins
const int tempsensor = A0;
const int currentsensor = A1;
const int alarm_left = 2;
const int alarm_right = 3;

const int brk_left = 4;
const int en_left = 5;
const int brk_right = 6;
const int en_right = 7;

float tempc;  //temperature in Celsius
//float tempf;
float vout;  //temporary variable to hold sensor reading


//HYPERPARAMETERS
//CHANGE TO DESIRED VALUES
const float MAX_SAFE_TEMP = 50;
const float MAX_SAFE_CUR = 3;

// variables for storage
float AcsValue = 0.0, Samples = 0.0, AvgAcs = 0.0, AcsValueF = 0.0;   // current
float tempC;

int temperature_error = 0;
int current_error = 0;
int alarm_error = 0;

// restart settings 
char received_msg;        // r for restart when ok, s for shutoff, n for don't auto-restart
int restart_when_ok = 0;
int motor_state = 0;      // 0 = shutoff, 1 = restart

// messages
char ok_msg[2] = "OK";
char current_msg[30] = "CURRENT";
char temperature_msg[30] = "TEMPERATURE";
char alarm_msg[30] = "ALARM";
char user_msg[30] = "USER";

void setup() {
  pinMode(tempsensor, INPUT);
  pinMode(currentsensor, INPUT);
  pinMode(alarm_left, INPUT);
  pinMode(alarm_right, INPUT);

  pinMode(brk_left, OUTPUT);
  pinMode(en_left, OUTPUT);
  pinMode(brk_left, OUTPUT);
  pinMode(en_right, OUTPUT);

  Serial.begin(115200);

  restart();
}

void loop() {
  // current
  
  for (int x = 0; x < 150; x++) {          //Get 150 samples
    AcsValue = analogRead(currentsensor);  //Read current sensor values
    Samples = Samples + AcsValue;          //Add samples together
    delay(3);                              // let ADC settle before next sample 3ms
  }
  AvgAcs = Samples / 150.0;
  AcsValueF = -1 * (2.5 - (AvgAcs * (5.0 / 1024.0)));
  //Taking Average of Samples
  //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
  //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
  //out to be 2.5 which is out offset. If your arduino is working on different voltage than
  //you must change the offset according to the input voltage)
  //0.100v(100mV) is rise in output voltage when 1A current flows at input

  //publish and activate alarm signal if necessary
  if (AcsValueF >= MAX_SAFE_CUR){
    if (!current_error) {
      current_error = 1;
      shutoff();
      Serial.println(current_msg);
    }
  }else{
    current_error = 0;
  }

  //calculating temperature
  vout = analogRead(tempsensor);  //Reading the value from sensor
  tempc = vout * (5.0 / 1023.0) * 10.0;

  if (tempc >= MAX_SAFE_TEMP) {
    if (!temperature_error){
      temperature_error = 1;
      shutoff();
      Serial.println(temperature_msg);
    }
  }else{
    temperature_error = 0;
  }

  // alarm signals
  if (digitalRead(alarm_left) || digitalRead(alarm_right)) {
    if (!alarm_error){
      alarm_error = 1;
      shutoff();
      Serial.println(alarm_msg);
    }
  } else {
    alarm_error = 0;
  }

  // serial input
  if (Serial.available() > 0) {
    received_msg = Serial.read();
    if (received_msg == 'r') {
      // enable motors
      restart_when_ok = 1;
    }else if (received_msg == 's') {
      // disable motors
      restart_when_ok = 0;
      shutoff();
      Serial.println(user_msg);
    }
  }

  // restart if allowed/ok, shutoff if needed
  if (!motor_state && restart_when_ok && !current_error && !temperature_error && !alarm_error){
    // everything ok + allowed to restart
    restart();
  }else if (motor_state && (current_error || temperature_error || alarm_error)){
    // something's wrong but motors are still on
    shutoff();
    if (current_error) Serial.println(current_msg);
    if (temperature_error) Serial.println(temperature_msg);
    if (alarm_error) Serial.println(alarm_msg);
  }else if (!motor_state && !restart_when_ok && !current_error && !temperature_error && !alarm_error){
    // everything ok + motors not enabled
    Serial.println(ok_msg);
  }
}

void shutoff(){
  digitalWrite(en_left, HIGH);
  digitalWrite(en_right, HIGH);
  digitalWrite(brk_left, HIGH);
  digitalWrite(brk_right, HIGH);

  motor_state = 0;
}

void restart(){
  digitalWrite(brk_left, LOW);
  digitalWrite(brk_right, LOW);
  digitalWrite(en_left, LOW);
  digitalWrite(en_right, LOW);

  motor_state = 1;

  Serial.println(ok_msg);
}