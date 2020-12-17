#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.2

// Servo range
#define _DUTY_MIN 900
#define _DUTY_NEU 1410
#define _DUTY_MAX 2100

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 300

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100
#define _RAMPUP_TIME 80 // servo speed rampup (0 to max) time (unit: ms)

//filter
#define LENGTH 30
#define k_LENGTH 8
#define Horizontal_Angle 1410
#define Max_Variable_Angle 110


// PID parameters
#define _KP 2.0
#define _KD 60.0
#define _KI 0.02


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;
// Distance sensor
float dist_target; // location to send the ball 
float dist_raw, dist_ema, dist_filtered;
const float coE[] = {-0.0000279, 0.0212565, -3.1466924, 237.7270779};

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_max;
int duty_chg_per_interval;
int duty_chg_adjust;
int toggle_interval, toggle_interval_cnt;
float pause_time;
int duty_target, duty_curr;

// PID variables
float error_curr = 0;
float error_prev = 0;
float control = 0;
float iterm = 0;
float pterm = 0;
float dterm = 0;

int correction_dist, iter;
float dist_list[LENGTH], sum, alpha;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
  dist_target = _DIST_TARGET;
  duty_curr = 1410;
  correction_dist = 0;
  iter = 0; sum = 0;
  alpha = _DIST_ALPHA;


// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
// initialize serial port
  Serial.begin(57600);
// servo related variables
  duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
  duty_chg_adjust = (float) duty_chg_max * _INTERVAL_SERVO / _RAMPUP_TIME;
  duty_chg_per_interval = 0; // initial speed is set to 0.

// initialize variables for duty_target update.
  pause_time = 0.5;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / _INTERVAL_SERVO;
  toggle_interval_cnt = toggle_interval;
  
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
    unsigned long time_curr = millis();
    if(millis() < last_sampling_time_dist + _INTERVAL_SERVO) return;

    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
          last_sampling_time_dist += _INTERVAL_DIST;
          event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
          last_sampling_time_servo += _INTERVAL_SERVO;
          event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
          last_sampling_time_serial += _INTERVAL_SERIAL;
          event_serial = true;
    }


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;
  // get a distance reading from the distance sensor
      dist_filtered = ir_distance_filtered();

  // PID control logic
    error_curr = _DIST_TARGET - dist_filtered;
    pterm = _KP * error_curr; 
    dterm = _KD *(error_curr-error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + (int)control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  // duty_target < _DUTY_MIN 일 때 duty_target 를 _DUTY_MIN 로 고정
    } else if (duty_target > _DUTY_MAX) {
       duty_target = _DUTY_MAX; // duty_target > _DUTY_MAX 일 때 duty_target 를 _DUTY_MAX 로 고정
    }
      error_prev = error_curr;  
    if (dist_filtered < _DIST_MIN) {
      dist_filtered = _DIST_MIN;
    } else if (dist_filtered > _DIST_MAX) {
      dist_filtered = _DIST_MAX;
    }
  }
  if(event_servo) {
   event_servo = false;
    
   // adjust duty_curr toward duty_target
   if(duty_target > duty_curr) {
     if(duty_chg_per_interval < duty_chg_max) {
       duty_chg_per_interval += duty_chg_adjust;
       if(duty_chg_per_interval > duty_chg_max) duty_chg_per_interval = duty_chg_max;
     }
     duty_curr += duty_chg_per_interval;
     if(duty_curr > duty_target) duty_curr = duty_target;
   }
   else if(duty_target < duty_curr) {
     if(duty_chg_per_interval > -duty_chg_max) {
       duty_chg_per_interval -= duty_chg_adjust;
       if(duty_chg_per_interval < -duty_chg_max) duty_chg_per_interval = -duty_chg_max;
     }
     duty_curr += duty_chg_per_interval;
     if(duty_curr < duty_target) duty_curr = duty_target;
   }
   else {
     duty_chg_per_interval = 0;
  }
}
  
// update servo position
  myservo.writeMicroseconds(duty_curr);

  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_filtered);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }

// toggle duty_target between _DUTY_MIN and _DUTY_MAX.
  if(toggle_interval_cnt >= toggle_interval) {
    toggle_interval_cnt = 0;
    if(duty_curr < _DUTY_MIN + 300) duty_target = _DUTY_MAX - 100;
    else if(duty_curr > _DUTY_MAX - 300) duty_target = _DUTY_MIN + 100;
  }
  else {
    toggle_interval_cnt++;
  }

// update last sampling time
  last_sampling_time_dist += _INTERVAL_SERVO;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float caliVal;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  caliVal = coE[0] * pow(val, 3) + coE[1] * pow(val, 2) + coE[2] * val + coE[3];
  return caliVal;
}

float ir_distance_filtered(void){ // return value unit: mm
  sum = 0;
  iter = 0;
  while (iter < LENGTH)
  {
    dist_list[iter] = ir_distance();
    sum += dist_list[iter];
    iter++;
  }

  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }
  
  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }

  float dist_cali = sum/(LENGTH-2*k_LENGTH)*1.05;
  float filt_val;
  filt_val = alpha*dist_cali + (1-alpha)*dist_filtered;
  return filt_val;
}
