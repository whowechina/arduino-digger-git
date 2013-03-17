
#define THROTTLE  0
#define ROLL      1
#define PITCH     2
#define YAW       3
#define PUSH      4

#define SPEED_THRESHOLD  20

#define OUT_LEFT_P       3
#define OUT_LEFT_N       5
#define OUT_RIGHT_P
#define OUT_RIGHT_N
#define OUT_SP
#define OUT_SN
#define OUT_DIG_P
#define OUT_DIG_N
#define OUT_PUSH
#define OUT_LED

static uint8_t failcount =0;
uint8_t rcfail;

int16_t ML_fwd, ML_bck, MR_fwd, MR_bck;

void setup()
{
  DDRC = 0x00;
  PINC = 0xff;
  PORTC = 0xff;

  PCMSK1 = B111111;
  PCICR = B00000010;
  
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  
  Serial.begin(115200);
}

uint16_t rc_raw[6];
int16_t rc[6];


#define PWM_TICK           \
  pwm_timer ++;            \
  if (pwm_timer >= 125)    \
    pwm_timer = 0;
  
#define PWM(pin, number)           \
  if (number > pwm_timer)          \
    digitalWrite(pin, HIGH);       \
  else                             \
    digitalWrite(pin, LOW); 


void loop()
{
  static uint8_t pwm_timer;
  
  PWM_TICK;

  computerc();
  movement();

  PWM(3, ML_fwd >> 1);
  PWM(4, ML_bck >> 1);
  
  Serial.print("a.b.\n");

#if 0  
  Serial.print(count);
  Serial.print(":");
  //Serial.print(failcount);
  //Serial.print(":");
  Serial.print(rcfail);
  Serial.print("-");
  Serial.print("ML+:");
  Serial.print(ML_fwd);
  Serial.print("ML-:");
  Serial.print(ML_bck);
  Serial.print("MR+:");
  Serial.print(MR_fwd);
  Serial.print("MR-:");
  Serial.print(MR_bck);
  Serial.println("");
#endif
}

#define RX_COMPUTE(pos)           \
  rc[pos] = ((rc_raw[pos] >> 2) - 375);

void computerc()
{
  static uint16_t oldrc[6];
  
  if ((oldrc[0] == rc_raw[0]) &&
      (oldrc[1] == rc_raw[1]) &&
      (oldrc[2] == rc_raw[2]) &&
      (oldrc[3] == rc_raw[3]) &&
      (oldrc[4] == rc_raw[4]))
  {
    /* All channels freeze */
    if (failcount < 50)
      failcount ++;
    else
      rcfail = 1;
  }
  else
  {
    /* Some channels is alive */
    if (failcount >10)
      failcount -= 10;
    else
      rcfail = 0;
      
    oldrc[0] = rc_raw[0];
    oldrc[1] = rc_raw[1];
    oldrc[2] = rc_raw[2];
    oldrc[3] = rc_raw[3];
    oldrc[4] = rc_raw[4];

    RX_COMPUTE(THROTTLE);
    RX_COMPUTE(ROLL);
    RX_COMPUTE(PITCH);
    RX_COMPUTE(YAW);
    RX_COMPUTE(PUSH);
    
  }
}

void movement()
{
  if (rc[THROTTLE] >= 0)
  {
    /* go forward: apply fwd speed to left and right motor */
    ML_fwd = (rc[THROTTLE] >> 1) + (rc[THROTTLE] << 1);    
    MR_fwd = ML_fwd;
    ML_bck = 0;
    MR_bck = 0;
    
    if (rc[ROLL] > 10)
      ML_fwd = min(ML_fwd * (100 - rc[ROLL]) / 100, ML_fwd); /* turn left: slow down the left motor */
    else if (rc[ROLL] < -10)
      MR_fwd = min(MR_fwd * (100 + rc[ROLL]) / 100, MR_fwd); /* turn right: slow down the right motor */
  }
  else
  {
    /* go backward: apply bck speed to left and right motor */
    ML_bck = ((- rc[THROTTLE]) >> 1) + ((- rc[THROTTLE]) << 1);
    MR_bck = ML_bck;
    ML_fwd = 0;
    MR_fwd = 0;
    
    if (rc[ROLL] > 10)
      ML_bck = min(ML_bck * (100 - rc[ROLL]) / 100, ML_bck); /* turn left: slow down the left motor */
    else if (rc[ROLL] < -10)
      MR_bck = min(MR_bck * (100 + rc[ROLL]) / 100, MR_bck); /* turn right: slow down the right motor */
  }
  
  /* too little speed or negative speed should be 0 */
  if (ML_fwd < SPEED_THRESHOLD)
    ML_fwd = 0;
  
  if (MR_fwd < SPEED_THRESHOLD)
    MR_fwd = 0;
  
  if (ML_bck < SPEED_THRESHOLD)
    ML_bck = 0;
  
  if (MR_bck < SPEED_THRESHOLD)
    MR_bck = 0;
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
    RX_CHECK(PUSH);
}
