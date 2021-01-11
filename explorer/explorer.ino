#define BMX055_DISABLE_BMM

/*#define FRONT_LEFT 6
#define FRONT_RIGHT 10
#define PWM_MIN 0
#define PWM_MAX 255
*/
#define M1_ENA 6
#define M2_ENB 5

#define M1_IN1 7
#define M1_IN2 8

#define M2_IN3 3
#define M2_IN4 4


#include <iarduino_Position_BMX055.h>
iarduino_Position_BMX055 sensor(BMX);

unsigned long prevTime = 4000; // задаем паузу перед началом движения
float prevCourse = 0.0; // задаем курс в градусах, по которому будем двигаться
int target_speed = 110; // скорость движения
int turnaround_speed = 90; // скорость разворота (минимальная)
int minOut = 0;
int maxOut = 200;
 
// параметры регулятора
float kp = 2.0;
float ki = 8.0;
float kd = 0.0;
int dt = 100;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  sensor.begin();
  sensor.setFastOffset();
  setup_motors_pin();
  stop_motors();
  
}

void loop() {
  unsigned long sensTime = millis();

  if (sensTime - prevTime > dt) {

    // считываем курс, измеренный IMU
    sensor.read();
    float heading = sensor.axisZ;

    // задаем курс
    float course = get_course(sensTime);
    
    if (course != prevCourse) {
      // разворот
      turnaround(heading, course);   
    }
    else {
      // едем прямо
      course_control(heading, course);
    }

    Serial.print(sensTime); Serial.print(", ");
    Serial.print(heading); Serial.print(", ");
    Serial.print(course); Serial.print(", ");
    

    
    prevTime = sensTime;
    prevCourse = course;    
  }

}

void setup_motors_pin() 
{
    pinMode(M1_ENA, OUTPUT);
    pinMode(M2_ENB, OUTPUT);
    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);
    pinMode(M2_IN3, OUTPUT);
    pinMode(M2_IN4, OUTPUT);
}

void stop_motors() 
{
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN3, LOW);
    digitalWrite(M2_IN4, LOW);
}

void control_motors(int pwm_l, int pwm_r) 
{
    uint8_t dir_l = 0;
    uint8_t dir_r = 0;
    if (pwm_l < 0) dir_l = 1;
    if (pwm_r < 0) dir_r = 1;
    
    digitalWrite(M1_IN1, !dir_l);
    digitalWrite(M1_IN2, dir_l);
    
    digitalWrite(M2_IN3, !dir_r);
    digitalWrite(M2_IN4, dir_r);
    
    analogWrite(M1_ENA, abs(pwm_l));
    analogWrite(M2_ENB, abs(pwm_r));
}

// функция ПИД регулятора
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float diff = (err - prevErr) / dt;
  prevErr = err;
  int output = err * kp + integral + diff * kd;
  Serial.print(", pid_out= ");Serial.print(output); Serial.print(", ");
  //return constrain(output, minOut, maxOut);
  return output;
    
}

// функция езды по прямой
void course_control(float heading, float course){

  int dir = 0; // направление подруливания: 1 = вправо, -1 = влево, 0 = ??? 
  int regulator;
  //if ((course == 180.0) && (heading < -90)) { 
  if (course - heading > 270) { 
    // для корректной работы регулятора heading должен быть больше курса на величину 360 + heading
    regulator = computePID(360 + heading, course, kp, ki, kd, dt, minOut, maxOut);
    //dir = 1;
    dir = -1;        
  }
  //else if ((course == -180.0) && (heading > 90)){
  else if (heading - course > 270){
    // для корректной работы регулятора heading должен быть меньше курса на величину abs(course) - heading
    regulator = computePID(heading - 360, course, kp, ki, kd, dt, minOut, maxOut);
    //dir = -1;
    dir = 1;    
  }
  else {
    regulator = computePID(heading, course, kp, ki, kd, dt, minOut, maxOut);
    if (heading > course) dir = -1; // готовимся подруливать влево
    else if (heading < course) dir = 1; // готовимся подруливать вправо
  }
    
  if (dir == -1) { // подруливаем влево 
    //analogWrite(FRONT_LEFT, constrain(target_speed - abs(regulator), minOut, maxOut));      
    //analogWrite(FRONT_RIGHT, constrain(target_speed + abs(regulator), minOut, maxOut));
    control_motors(constrain(target_speed + abs(regulator), minOut, maxOut), constrain(target_speed - abs(regulator), minOut, maxOut));
    
  }
  else if (dir == 1) { // подруливаем вправо
    //analogWrite(FRONT_LEFT, constrain(target_speed + 0.5*abs(regulator), minOut, maxOut));
    //analogWrite(FRONT_RIGHT, constrain(target_speed - 0.5*abs(regulator), minOut, maxOut));
    control_motors(constrain(target_speed - abs(regulator), minOut, maxOut), constrain(target_speed + abs(regulator), minOut, maxOut));
  }

  // отладка:
  Serial.print(constrain(target_speed - abs(regulator), minOut, maxOut)); Serial.print(", ");
  Serial.println(constrain(target_speed + abs(regulator), minOut, maxOut));// Serial.print(", ");
}

// функция разворота
void turnaround(float heading, float course){
  while(abs(heading - course) > 5.0){
    Serial.print("Turnaround: ");
    if (abs(course) < 1){
      // поворачиваем влево
      //analogWrite(FRONT_LEFT, 0);      
      //analogWrite(FRONT_RIGHT, turnaround_speed);
      // поворачиваем вправо
      control_motors(turnaround_speed, 0);
    }
    else if (abs(course)>179.0) {
      // поворачиваем вправо
      //analogWrite(FRONT_LEFT, turnaround_speed);      
      //analogWrite(FRONT_RIGHT, 0);
      // поворачиваем влево
      control_motors(0, turnaround_speed);
    }
    sensor.read();
    heading = sensor.axisZ;
    
    Serial.print(heading); Serial.print("->");
    Serial.println(course);// Serial.print(", ");
  }
  // остановка, пауза, движение вперед
  //analogWrite(FRONT_LEFT, 0);      
  //analogWrite(FRONT_RIGHT, 0);
  stop_motors();
  delay(500);
  //control_motors(target_speed, target_speed);
  //delay(500);
  
  /*unsigned long sensTime = millis();
  while (millis() < sensTime + 500) {}
  analogWrite(FRONT_LEFT, target_speed);      
  analogWrite(FRONT_RIGHT, target_speed);
  sensTime = millis();
  while (millis() < sensTime + 500) {}
  */
}

// функция целеуказания
float get_course(unsigned long timer){

  float new_course;
  
  if ((timer / 5000) == 0) new_course = 0.0;
  else new_course = 180.0;
  Serial.print("new_course = ");Serial.print(new_course);

  return new_course;
}
