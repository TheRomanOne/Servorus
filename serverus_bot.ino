#include<Servo.h>

#define __THRESHOLD__ 10
#define __SERVO_PIN__ 7
#define __SENSOR1_PIN__ A0
#define __SENSOR2_PIN__ A1
#define __ROTATION_SPEED__ 2.5
#define __LIGHT_RANGE__ 400
#define __INIT_COUNT__ 2000
#define __MEMORY_SIZE__ 50
// #define __LED_PIN__ 4 //uncomment if led is added

void printf(double a, double b) { Serial.print(a); Serial.print(", "); Serial.println(b); }
void printf(double a, double b, double c) { Serial.print(a); Serial.print(", "); Serial.print(b); Serial.print(", "); Serial.println(c); }

int sign(float a) { return int(a/abs(a)); }


class Memory
{
  public:
    float sensor0[__MEMORY_SIZE__];
    float sensor1[__MEMORY_SIZE__];
    float angle[__MEMORY_SIZE__];
    float s0_avg, s1_avg, angle_avg;
  void add_sensor_data(float s0, float s1)
  {
    s0_avg = s0;
    s1_avg = s1;

    for(int i = __MEMORY_SIZE__ - 1; i > 0; i--)
    {
      s0_avg += sensor0[i];
      sensor0[i] = sensor0[i-1];
      s1_avg += sensor1[i];
      sensor1[i] = sensor1[i-1];
    }
    sensor0[0] = s0;
    sensor1[0] = s1;

    s0_avg /= __MEMORY_SIZE__;
    s1_avg /= __MEMORY_SIZE__;
  };

  void add_angle_data(float a)
  {
    for(int i = __MEMORY_SIZE__ - 1; i > 0; i--)
      angle[i] = angle[i-1];
    angle[0] = a;
  };


};
Servo servo;
int current_iter;
double min0_v, anbient0,
       min1_v, anbient1,
       prev_rot;

float prev00, prev01, prev02,
      prev10, prev11, prev12,
      rot0, rot1, rot2;

Memory* memory = new Memory();

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
   anbient0 += s0;
   anbient1 += s1;
   current_iter += 1;
  }else if(current_iter == __INIT_COUNT__)
  {
    // Do once
    
    // calculate min light intensity
    anbient0 /= __INIT_COUNT__;
    anbient1 /= __INIT_COUNT__;

    // Signal readiness. uncomment if led is added
    // digitalWrite(__LED_PIN__, HIGH);

    current_iter += 1;
    Serial.print("INIT: ");
    printf(anbient0, anbient1);
  }else 
  {
    // Working mode
    // calculate ervo angle
    float a = calculate_angle(s0, s1);
    servo.write(a);
    // delay(50);
  }  
}

double calculate_angle(double s0, double s1)
{
  // stabelize sensor read
  s0 = min(1, max(0, 2.*(anbient0 - s0) / __LIGHT_RANGE__));
  s1 = min(1, max(0, 2.*(anbient1 - s1) / __LIGHT_RANGE__));
  bool check1 = true;//
  memory->add_sensor_data(s0, s1);
  // range = [-1, 1]
  double a = memory->s0_avg - memory->s1_avg;
  bool check2 = s0 + s1 < .1;
  if(check2) return 90.;
  // enhance range
  a *= __ROTATION_SPEED__;
  // transform range to [0, 1]
  a += 1;
  a *= 90; // devide by 2, multiply by 180 angle
  a = int(a);
  printf(s0, s1, a);

  // Serial.println(a);
  if(check1)
  {
    prev_rot = a;
    return a;
  }
  return prev_rot;
}
