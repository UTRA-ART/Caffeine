// constants
const int HALL_PIN_L = 2;
const int HALL_PIN_R = 3;

const int BAUD_RATE = 115200;
const int PRINT_RATE = 30;

// counts
volatile uint_fast32_t pulse_count_l = 0;
volatile uint_fast32_t pulse_count_r = 0;

unsigned long last_print_time = 0;
unsigned long curr_print_time = 0;

// states
volatile bool prev_state_l = false;
volatile bool prev_state_r = false;
volatile bool curr_state_l = false;
volatile bool curr_state_r = false;

void setup(){
  pinMode(HALL_PIN_L, INPUT);
  pinMode(HALL_PIN_R, INPUT);

  Serial.begin(BAUD_RATE);
}

void loop(){
  // read states and update counts
  curr_state_l = digitalRead(HALL_PIN_L);
  if(curr_state_l && !prev_state_l) pulse_count_l++;
  prev_state_l = curr_state_l;

  curr_state_r = digitalRead(HALL_PIN_R);
  if(curr_state_r && !prev_state_r) pulse_count_r++;
  prev_state_r = curr_state_r;

  // print result
  curr_print_time = millis();
  if(curr_print_time - last_print_time > PRINT_RATE){
    Serial.print("<");
    Serial.print(pulse_count_l);
    Serial.print(",");
    Serial.print(pulse_count_r);
    Serial.println(">");

    last_print_time = curr_print_time;

    pulse_count_l = 0;
    pulse_count_r = 0;
  }
}
