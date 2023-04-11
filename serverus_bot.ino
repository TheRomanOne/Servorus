#include<Servo.h>

const int SERVO_PIN = 7;
const int SENSOR1_PIN = A0;
const int SENSOR2_PIN = A1;
const int LED_PIN = 4;
const double SMOOTHSTEP = .1;
const double ROTATION_SPEED = 2.;
const double LIGHT_INTENSITY = 350.;
const int NUM_OF_INIT_ITERS = 2000;

Servo servo;
int current_iter;
double min0_v, range0, prev0,
       min1_v, range1, prev1,
       prev_rot;

void printf(double a) { Serial.println(a); }
void printf(double a, double b) { Serial.print(a); Serial.print(", "); Serial.println(b); }
void printf(double a, double b, double c) { Serial.print(a); Serial.print(", "); Serial.print(b); Serial.print(", "); Serial.println(c); }
void printf(char a[], double b) { Serial.print(a); Serial.print(", "); Serial.println(b); }
void printf(char a[], double b, double c) { Serial.print(a); Serial.print(": "); Serial.print(b); Serial.print(", "); Serial.println(c); }

void setup()
{
  int current_iter = 0;
  double prev0 = 0;
  double prev1 = 0;

  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  pinMode(LED_PIN, OUTPUT);

  prev_rot = 90;
  servo.write(prev_rot);
  delay(1000);
}

void loop()
{
  double s0 = analogRead(SENSOR1_PIN);
  double s1 = analogRead(SENSOR2_PIN);
  
  if(current_iter < NUM_OF_INIT_ITERS)
  {
   // Init mode
   // add to avg calc
   min0_v += s0;
   min1_v += s1;
   current_iter += 1;
  }else if(current_iter == NUM_OF_INIT_ITERS)
  {
    // Do once
    
    // calculate min light
    min0_v /= NUM_OF_INIT_ITERS;
    min1_v /= NUM_OF_INIT_ITERS;

    range0 = LIGHT_INTENSITY - min0_v;
    range1 = LIGHT_INTENSITY - min1_v;

    // Signal readiness
    digitalWrite(LED_PIN, HIGH);
    current_iter += 1;
    Serial.print("INIT: ");
    printf(min0_v, min1_v);
  }else 
  {
    // Working mode
    
    s0 = min(1, max(0, (s0 - min0_v)/range0));
    s1 = min(1, max(0, (s1 - min1_v)/range1));
    
    s0 = (abs(s0-prev0) > SMOOTHSTEP) ? prev0: s0; 
    s1 = (abs(s1-prev1) > SMOOTHSTEP) ? prev1: s1; 
    
    double r = calculate_rotation(s0, s1);
    printf(s0, s1,r);
    servo.write(r);
    
    prev0 = s0;
    prev1 = s1;
  }  
}

double calculate_rotation(double s0, double s1)
{
  // range = [-1, 1]
  double r = min(max(s0, 0), 1)-min(max(s1, 0), 1);
  // enhance range
  // transform range to [0, 1]
  r *= ROTATION_SPEED;
  r += 1;
  r *= 90; // devide by 2, multiply by 180 angle
  r = int(r*10)/10.;
  if(abs(prev_rot - r) > 1)
  {
    prev_rot = r;
    return r;
  }
  return prev_rot;
}
