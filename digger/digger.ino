
#define THROTTLE  0
#define ROLL      1
#define PITCH     2
#define YAW       3
#define MODE      4

#define SPEED_THRESHOLD  40

#define PIN_ML_fwd       2
#define PIN_ML_bck       3
#define PIN_MR_fwd       4
#define PIN_MR_bck       5
#define PIN_SPIN_cw      6
#define PIN_SPIN_ccw     7
#define PIN_DIG_down     8
#define PIN_DIG_up       9
#define PIN_PUSH_fwd    10
#define PIN_LED_x       11


uint16_t rc_raw[6];
int16_t rc[6];
uint16_t now_ms;

uint8_t failcount =0;
uint8_t rcfail = 1;
uint8_t activated = 0;

int16_t ML_fwd = 0, ML_bck = 0, MR_fwd = 0, MR_bck = 0;
int16_t SPIN_cw = 0, SPIN_ccw = 0, DIG_down = 0, DIG_up = 0, PUSH_fwd = 0, LED_x = 0;

#define ZERO_OUTPUT  \
  ML_fwd = 0;        \
  ML_bck = 0;        \
  MR_fwd = 0;        \
  MR_bck = 0;        \
  SPIN_cw = 0;       \
  SPIN_ccw = 0;      \
  DIG_down = 0;      \
  DIG_up = 0;        \
  PUSH_fwd = 0;      \
  LED_x = 0;
  
void setup()
{
  /* Port C is for input and internal pull-up enabled */
  DDRC = 0x00;
  PINC = 0xff;
  PORTC = 0xff;

  /* Enable interrupts from 5 pins of port C which are connected with RC */
  PCMSK1 = B11111;
  PCICR = B00000010;
  
  /* Disable internal pull-up for output pins */
  digitalWrite(PIN_ML_fwd, LOW);
  digitalWrite(PIN_ML_bck, LOW);
  digitalWrite(PIN_MR_fwd, LOW);
  digitalWrite(PIN_MR_bck, LOW);
  digitalWrite(PIN_SPIN_cw, LOW);
  digitalWrite(PIN_SPIN_ccw, LOW);
  digitalWrite(PIN_DIG_down, LOW);
  digitalWrite(PIN_DIG_up, LOW);
  digitalWrite(PIN_PUSH_fwd, LOW);
  digitalWrite(PIN_LED_x, LOW);

  pinMode(PIN_ML_fwd, OUTPUT);
  pinMode(PIN_ML_bck, OUTPUT);
  pinMode(PIN_MR_fwd, OUTPUT);
  pinMode(PIN_MR_bck, OUTPUT);
  pinMode(PIN_SPIN_cw, OUTPUT);
  pinMode(PIN_SPIN_ccw, OUTPUT);
  pinMode(PIN_DIG_down, OUTPUT);
  pinMode(PIN_DIG_up, OUTPUT);
  pinMode(PIN_PUSH_fwd, OUTPUT);
  pinMode(PIN_LED_x, OUTPUT);

  Serial.begin(115200);
}

  static uint8_t acttick = 0;

void loop()
{
  static uint16_t old;
  
  now_ms = millis();
  
  computerc();
  control();
  finetune();
  outputpwm();
  
  if (now_ms - old >= 100)
  {
    old = now_ms;
    Serial.print(failcount);
    Serial.print("-");
    Serial.print(rcfail);
    Serial.print("-");
    Serial.print(activated);
    Serial.print("-");
    Serial.print(acttick);
    Serial.print("-");
    Serial.print(" ML+:");
    Serial.print(ML_fwd);
    Serial.print(" ML-:");
    Serial.print(ML_bck);
    Serial.print(" MR+:");
    Serial.print(MR_fwd);
    Serial.print(" MR-:");
    Serial.print(MR_bck);
    Serial.print(" SP+:");
    Serial.print(SPIN_cw);
    Serial.print(" SP-:");
    Serial.print(SPIN_ccw);
    Serial.print(" DIG+:");
    Serial.print(DIG_down);
    Serial.print(" DIG-:");
    Serial.print(DIG_up);
    Serial.print(" PSH:");
    Serial.print(PUSH_fwd);
    Serial.print(" LED:");
    Serial.print(LED_x);
    Serial.print("\t");
    Serial.print(rc[0]);
    Serial.print(":");
    Serial.print(rc[1]);
    Serial.print(":");
    Serial.print(rc[2]);
    Serial.print(":");
    Serial.print(rc[3]);
    Serial.print(":");
    Serial.print(rc[4]);
    Serial.println();
  }
}

#define RX_COMPUTE(pos)                 \
  rc[pos] = ((rc_raw[pos] >> 2) - 375);

void computerc()
{
  static uint16_t oldrc[6];
  static uint16_t old_ms;
  
  if (now_ms - old_ms > 20)
  {
    old_ms = now_ms;
    if ((oldrc[1] == rc_raw[1]) &&
        (oldrc[2] == rc_raw[2]) &&
        (oldrc[3] == rc_raw[3]) &&
        (oldrc[4] == rc_raw[4]))
    {
      /* All channels freeze */
      if (failcount < 50)
        failcount ++;
      else
      {
        rcfail = 1;
        activated = 0;
      }
    }
    else
    {
      /* Some channels is alive */
      if (failcount >10)
        failcount -= 10;
      else
        rcfail = 0;
    }
    oldrc[1] = rc_raw[1];
    oldrc[2] = rc_raw[2];
    oldrc[3] = rc_raw[3];
    oldrc[4] = rc_raw[4];
  }

  RX_COMPUTE(THROTTLE);
  RX_COMPUTE(ROLL);
  RX_COMPUTE(PITCH);
  RX_COMPUTE(YAW);
  RX_COMPUTE(MODE);
}

void control()
{
  static uint8_t led_can_switch;
  
  if (rcfail)
  {
    ZERO_OUTPUT;
    delay(5);
    return;
  }
  
  /* Activation decision */
  if (! activated)
  {
    ZERO_OUTPUT;

    if ((rc[THROTTLE] >= -15) && (rc[THROTTLE] <= 15))
      acttick ++;
    else
      acttick = 0;

    if (acttick >= 200)
    {
      acttick = 0;
      activated = 1;
      LED_x = 0xff; /* it's good to light up LED when activated */
    }
    delay(10);
    return;
  }
  
  /* STEP1: Main motors with steering control */
  if (rc[THROTTLE] >= 0)
  {
    /* go forward: apply fwd speed to left and right motor */
    ML_fwd = (rc[THROTTLE] >> 1) + (rc[THROTTLE] << 1);    
    MR_fwd = ML_fwd;
    ML_bck = 0;
    MR_bck = 0;
    
    if (rc[YAW] > 10)
      ML_fwd = min(ML_fwd * (100 - rc[YAW]) / 100, ML_fwd); /* turn left: slow down the left motor */
    else if (rc[YAW] < -10)
      MR_fwd = min(MR_fwd * (100 + rc[YAW]) / 100, MR_fwd); /* turn right: slow down the right motor */
  }
  else
  {
    /* go backward: apply bck speed to left and right motor */
    ML_bck = ((- rc[THROTTLE]) >> 1) + ((- rc[THROTTLE]) << 1);
    MR_bck = ML_bck;
    ML_fwd = 0;
    MR_fwd = 0;
    
    if (rc[YAW] > 10)
      ML_bck = min(ML_bck * (100 - rc[YAW]) / 100, ML_bck); /* turn left: slow down the left motor */
    else if (rc[YAW] < -10)
      MR_bck = min(MR_bck * (100 + rc[YAW]) / 100, MR_bck); /* turn right: slow down the right motor */
  }
  
  /* STEP2: Spin motor */
  if (rc[ROLL] >= 0)
  {
    /* spin counter-clock-wise */
    SPIN_cw = (rc[ROLL] >> 1) + (rc[ROLL] << 1);    
    SPIN_ccw = 0;
  }
  else
  {
    /* spin clock wise */
    SPIN_ccw = ((- rc[ROLL]) >> 1) + ((- rc[ROLL]) << 1);
    SPIN_cw = 0;
  }
  
  /* STEP3: Two modes */
  if (rc[MODE] > 0)
  {
    /* Mode 1: Pitch controls digging */
    if (rc[PITCH] >= 0)
    {
      /* spin counter-clock-wise */
      DIG_down = (rc[PITCH] >> 1) + (rc[PITCH] << 1);    
      DIG_up = 0;
    }
    else
    {
      /* spin clock wise */
      DIG_up = ((- rc[PITCH]) >> 1) + ((- rc[PITCH]) << 1);
      DIG_down = 0;
    }
  }
  else
  {
    /* Mode 2: Pitch up to push, pitch down to turn on/off led */
    if (rc[PITCH] >= 0)
    {
      /* spin counter-clock-wise */
      PUSH_fwd = (rc[PITCH] >> 1) + (rc[PITCH] << 1);
    }
    else
    {
      /* A schmit trigger for toggling LED */
      if (rc[PITCH] < -90)
      {
        if (led_can_switch)
        {
          if (LED_x > 0)
            LED_x = 0;
          else
            LED_x = 0xff;
          led_can_switch = 0;
        }
      }
      if (rc[PITCH] > -20)
      {
        led_can_switch = 1;
      }
    }
  }
  
}

#define THRESHOLD_CHECK(motor)   \
  if (motor < SPEED_THRESHOLD)   \
    motor = 0;

void finetune()
{
  /* Trim too little speed or negative speed */
  THRESHOLD_CHECK(ML_fwd);
  THRESHOLD_CHECK(ML_bck);
  THRESHOLD_CHECK(MR_fwd);
  THRESHOLD_CHECK(MR_bck);
  THRESHOLD_CHECK(SPIN_cw);
  THRESHOLD_CHECK(SPIN_ccw);
  THRESHOLD_CHECK(DIG_up);
  THRESHOLD_CHECK(DIG_down);
  THRESHOLD_CHECK(PUSH_fwd);
  
  /* Protection */
  if ((ML_fwd > 0) && (ML_bck > 0))
  {
    ML_fwd = 0;
    ML_bck = 0;
  }
  if ((MR_fwd > 0) && (MR_bck > 0))
  {
    MR_fwd = 0;
    MR_bck = 0;
  }
  if ((SPIN_cw > 0) && (SPIN_ccw > 0))
  {
    SPIN_cw = 0;
    SPIN_ccw = 0;
  }
  if ((DIG_up > 0) && (DIG_down > 0))
  {
    DIG_up = 0;
    DIG_down = 0;
  }
}

#define PWM_TICK           \
  pwm_timer ++;            \
  if (pwm_timer >= 115)    \
    pwm_timer = 0;
  
#define PWM_ZERO(pin, number)      \
  if ((number) == 0)               \
    digitalWrite(pin, LOW); 

#define PWM_NONZERO(pin, number)      \
  if ((number) > (pwm_timer) + 10)    \
    digitalWrite(pin, HIGH);          \
  else                                \
    digitalWrite(pin, LOW); 

void outputpwm()
{
  static uint8_t pwm_timer;
  
  PWM_TICK;

  PWM_ZERO(PIN_ML_fwd, ML_fwd >> 1);
  PWM_ZERO(PIN_ML_bck, ML_bck >> 1);
  PWM_ZERO(PIN_MR_fwd, MR_fwd >> 1);
  PWM_ZERO(PIN_MR_bck, MR_bck >> 1);
  PWM_ZERO(PIN_SPIN_cw, SPIN_cw >> 1);
  PWM_ZERO(PIN_SPIN_ccw, SPIN_ccw >> 1);
  PWM_ZERO(PIN_DIG_down, DIG_down >> 1);
  PWM_ZERO(PIN_DIG_up, DIG_up >> 1);
  PWM_ZERO(PIN_PUSH_fwd, PUSH_fwd >> 1);
  PWM_ZERO(PIN_LED_x, LED_x >> 1);
  
  PWM_NONZERO(PIN_ML_fwd, ML_fwd >> 1);
  PWM_NONZERO(PIN_ML_bck, ML_bck >> 1);
  PWM_NONZERO(PIN_MR_fwd, MR_fwd >> 1);
  PWM_NONZERO(PIN_MR_bck, MR_bck >> 1);
  PWM_NONZERO(PIN_SPIN_cw, SPIN_cw >> 1);
  PWM_NONZERO(PIN_SPIN_ccw, SPIN_ccw >> 1);
  PWM_NONZERO(PIN_DIG_down, DIG_down >> 1);
  PWM_NONZERO(PIN_DIG_up, DIG_up >> 1);
  PWM_NONZERO(PIN_PUSH_fwd, PUSH_fwd >> 1);
  PWM_NONZERO(PIN_LED_x, LED_x >> 1);
}

#define RX_CHECK(pos)                         \
  if (mask & (1 << pos))                      \
  {                                           \
    if (pin & (1 << pos))                     \
      start[pos] = now;                       \
    else                                      \
    {                                         \
      delta = now - start[pos];               \
      if ((1000 < delta) && (delta < 2000))   \
      {                                       \
        rc_raw[pos] = delta;                  \
      }                                       \
    }                                         \
  }

ISR(PCINT1_vect)
{ //this ISR is common to every receiver channel, it is call everytime a change state occurs on a RX input pin
    uint8_t mask;
    uint8_t pin;
    uint16_t now, delta;
    static uint16_t start[5];
    static uint8_t last;

    pin = PINC; // RX_PCINT_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins
   
    mask = pin ^ last;
    now = micros();
    sei();
    last = pin;

    RX_CHECK(THROTTLE);
    RX_CHECK(ROLL);
    RX_CHECK(PITCH);
    RX_CHECK(YAW);
    RX_CHECK(MODE);
}
