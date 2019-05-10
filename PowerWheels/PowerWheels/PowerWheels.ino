/*
  PowerWheels

  Slow start of power wheels and de-rate motor drive for higher-voltage battery (Ryobi One+ 18V).

  The circuit:
  - PWM L - Pin 2
  - PWM R - Pin 3
  - Phase L EN - Pin 5 
  - Phase R EN - Pin 6
  - Relay - Pin 7
  - Direction - Pin 8
  - Speed - Pin 9

  created 4 Apr 2018
  by Devan Lippman
*/

// OUTPUT PIN DEFINITIONS
#define L_EN_PIN  (5)
#define L_PWM_PIN (2)
#define R_EN_PIN  (6)
#define R_PWM_PIN (3)
#define RELAY_PIN (7)

//INPUT PIN DEFINITIONS
#define DIRECTION_PIN (8)
#define SPEED_PIN     (9)

//GLOBAL VARIABLES
uint32_t dutyCycle = 0;
uint32_t last_micros;
int last_direction = -1;
uint32_t period_start;
bool directionState = false;
bool speedState = false;

//CONSTANT DEFINITIONS
#define STEP_PERIOD_US      (16666UL / 2)
#define STEP_INC_PERIOD_US  (STEP_PERIOD_US * 2)
#define MAX_DUTY_LIMIT      (STEP_PERIOD_US) 
#define SLOW_DUTY_LIMIT     (MAX_DUTY_LIMIT * 2 / 3)
#define STARTING_DUTY       (SLOW_DUTY_LIMIT / 2)

//(int(STEP_PERIOD_US*12.0/18.0))

void debounceInputs(void)
{
  bool reading;
  
  enum{
    INPUT_DIR,
    INPUT_SPEED,
    INPUT_CNT
  };
  
  static struct {
    uint32_t count;
    bool last_read;
    uint8_t pin_num;
    bool* signal_ptr;  
  } signals[2] = {0,false,DIRECTION_PIN,&directionState,0,false,SPEED_PIN,&speedState};

  for(uint32_t i = 0; i < INPUT_CNT; i++) {
     reading = digitalRead(signals[i].pin_num);
     if (reading != *(signals[i].signal_ptr)){
       if (signals[i].last_read != reading) {
         signals[i].count = 0;
       }
       else if (signals[i].count > 5) {
         *(signals[i].signal_ptr) = reading;
       }
       else {
         signals[i].count += 1;
       }
     }
     signals[i].last_read = reading;
  }
}

void setup() {
  //Configure Outputs
  pinMode(L_EN_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  
  digitalWrite(L_EN_PIN, LOW);
  digitalWrite(L_PWM_PIN, LOW);
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(R_PWM_PIN, LOW);
  digitalWrite(RELAY_PIN, HIGH);

  //Configure Inputs
  pinMode(DIRECTION_PIN, INPUT_PULLUP);
  pinMode(SPEED_PIN, INPUT_PULLUP);

  //Initialize Globals
  last_micros = micros();
  period_start = last_micros;
  directionState = digitalRead(DIRECTION_PIN);
  speedState = digitalRead(SPEED_PIN);
  
  Serial.begin(9600);
  
}
  
unsigned int get_speed_limit(bool turbo) 
{
  return turbo?MAX_DUTY_LIMIT:SLOW_DUTY_LIMIT;
}

void ledTx( boolean on)
{
  if( on)
  {
    // led on. The led is connected to VCC. Make pin low to turn led on.
    pinMode( LED_BUILTIN_TX, OUTPUT);    // pin as output.
    digitalWrite( LED_BUILTIN_TX, LOW);  // pin low

    // These two lines will do the same:
    //    bitSet( DDRD, 5);         
    //    bitClear( PORTD, 5);       
  }
  else
  {
    // led off
    // turn it off, by setting it as input, so the serial activity can't turn it on.
    // If the internal pullup resistor is enabled or not, that does not matter,
    // since the led it connected to VCC.
    pinMode( LED_BUILTIN_TX, INPUT);

    // This line will do the same:
    //    bitClear( DDRD, 5);        // set pin as input
  }
}

void digitalToggle(uint8_t pin)
{
  digitalWrite(pin, !digitalRead(pin));
}

void loop() {
  debounceInputs();
  uint32_t now = micros();
  static uint8_t step_cnt = 0;

  //Enable H-Bridge Drivers
  digitalWrite(L_EN_PIN, HIGH);
  digitalWrite(R_EN_PIN, HIGH);

  //Handle new direction (Fwd/Rev)
  if(last_direction == -1 || last_direction != directionState) {
    last_direction = directionState;
    dutyCycle = STARTING_DUTY;
    last_micros = now;
  }

  //Ramp acceleration
  //if (((now - last_micros) >= STEP_INC_PERIOD_US)) {
  if (step_cnt >= 2) {
    dutyCycle += STEP_PERIOD_US/255;
    last_micros = now;
    step_cnt = 0;
  } 

  //Cap speed/duty
  if (dutyCycle >= (speedState?MAX_DUTY_LIMIT:SLOW_DUTY_LIMIT)) {
    dutyCycle = speedState?MAX_DUTY_LIMIT:SLOW_DUTY_LIMIT;
  }

  //Bit-bang PWM (H-Bridge sign-magnitude drive)
  if (((now - period_start) < dutyCycle)) {
    //High time handled here
    digitalWrite(R_PWM_PIN, directionState);
    digitalWrite(L_PWM_PIN, !directionState);
  }
  else if((now - period_start) >= STEP_PERIOD_US) {
      period_start = now;
      step_cnt++;
  }
  else {
    //Low time handled here
    digitalWrite(R_PWM_PIN, HIGH);
    digitalWrite(L_PWM_PIN, HIGH);
  }
}
