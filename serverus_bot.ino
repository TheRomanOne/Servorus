#include<Servo.h>

#define __THRESHOLD__ 3
#define __SERVO_PIN__ 7
#define __SENSOR1_PIN__ A0
#define __SENSOR2_PIN__ A1
#define __ROTATION_SPEED__ 2.
#define __LIGHT_DAMP__ 420.
#define __INIT_COUNT__ 2000
// #define __LED_PIN__ 4 //uncomment if led is added

Servo servo;
int current_iter;
double min0_v, anbient0,
       min1_v, anbient1,
       prev_rot;

void printf(double a, double b) { Serial.print(a); Serial.print(", "); Serial.println(b); }
void printf(double a, double b, double c) { Serial.print(a); Serial.print(", "); Serial.print(b); Serial.print(", "); Serial.println(c); }

void setup()
{
  prev_rot = 90;

  Serial.begin(9600);
  servo.attach(__SERVO_PIN__);
  // pinMode(__LED_PIN__, OUTPUT); //uncomment if led is added

  servo.write(prev_rot);
  delay(1000);
}

void loop()
{
  // Get sensor reading
  double s0 = analogRead(__SENSOR1_PIN__);
  double s1 = analogRead(__SENSOR2_PIN__);
  
  if(current_iter < __INIT_COUNT__)
  {
   // Init mode
   // add to calc light intensity avg
   min0_v += s0;
   min1_v += s1;
   current_iter += 1;
  }else if(current_iter == __INIT_COUNT__)
  {
    // Do once
    
    // calculate min light intensity
    min0_v /= __INIT_COUNT__;
    min1_v /= __INIT_COUNT__;

    // calc ambiant light
    anbient0 = __LIGHT_DAMP__ - min0_v;
    anbient1 = __LIGHT_DAMP__ - min1_v;

    // Signal readiness. uncomment if led is added
    // digitalWrite(__LED_PIN__, HIGH);

    current_iter += 1;
    Serial.print("INIT: ");
    printf(min0_v, min1_v);
  }else 
  {
    // Working mode
    
    // calculate ervo angle
    double a = calculate_angle(s0, s1);
    servo.write(a);
    
  }  
}

double calculate_angle(double s0, double s1)
{
  // stabelize sensor read
  s0 = min(1, max(0, ((int(s0*10)/10.) - min0_v)/anbient0));
  s1 = min(1, max(0, ((int(s1*10)/10.) - min1_v)/anbient1));
  
  // range = [-1, 1]
  double a = s0 - s1;
  // enhance range
  a *= __ROTATION_SPEED__;
  // transform range to [0, 1]
  a += 1;
  a *= 90; // devide by 2, multiply by 180 angle
  
  // return smoothe signal
  a = int(a*10)/10.;
  if(abs(prev_rot - a) > __THRESHOLD__)
  {
    prev_rot = a;
    printf(s0, s1,a);
    return a;
  }
  return prev_rot;
}
