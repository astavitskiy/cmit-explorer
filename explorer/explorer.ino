#define BMX055_DISABLE_BMM

#define FRONT_LEFT 6
#define FRONT_RIGHT 10
#define PWM_MIN 0
#define PWM_MAX 255

#include <iarduino_Position_BMX055.h>
iarduino_Position_BMX055 sensor(BMX);

unsigned long prevTime = 4000; // задаем паузу перед началом движения
float prevCourse = 0.0; // задаем курс в градусах, по которому будем двигаться
int target_speed = 150; // скорость движения
int turnaround_speed = 90; // скорость разворота (минимальная)
int minOut = 0;
int maxOut = 200;
 
// параметры регулятора
float kp = 10.0;
float ki = 0.0;
float kd = 0.0;
int dt = 10;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  sensor.begin();
  sensor.setFastOffset();
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

// функция ПИД регулятора
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float diff = (err - prevErr) / dt;
  prevErr = err;
  int output = err * kp + integral + diff * kd;
  Serial.print("out= ");Serial.print(output); Serial.print(", ");
  //return constrain(output, minOut, maxOut);
  return output;
    
}

// функция езды по прямой
void course_control(float heading, float course){

  int dir = 0; // направление подруливания: 1 = вправо, -1 = влево, 0 = ??? 
  int regulator;
  if ((course == 180.0) && (heading < -90)) { 
    // для корректной работы регулятора heading должен быть больше курса на величину abs(heading) - course
    regulator = computePID(course + (abs(heading) - course), course, kp, ki, kd, dt, minOut, maxOut);
    dir = 1;    
  }
  else if ((course == -180.0) && (heading > 90)){
    // для корректной работы регулятора heading должен быть меньше курса на величину abs(course) - heading
    regulator = computePID(course - (abs(course) - heading), course, kp, ki, kd, dt, minOut, maxOut);
    dir = -1;
  }
  else {
    regulator = computePID(heading, course, kp, ki, kd, dt, minOut, maxOut);
    if (heading > course) dir = -1; // готовимся подруливать влево
    else if (heading < course) dir = 1; // готовимся подруливать вправо
  }
    
  if (dir == -1) { // подруливаем влево 
    analogWrite(FRONT_LEFT, constrain(target_speed - abs(regulator), minOut, maxOut));      
    analogWrite(FRONT_RIGHT, constrain(target_speed + abs(regulator), minOut, maxOut));
  }
  else if (dir == 1) { // подруливаем вправо
    analogWrite(FRONT_LEFT, constrain(target_speed + 0.5*abs(regulator), minOut, maxOut));
    analogWrite(FRONT_RIGHT, constrain(target_speed - 0.5*abs(regulator), minOut, maxOut));
  }

  // отладка:
  Serial.print(constrain(target_speed - abs(regulator), minOut, maxOut)); Serial.print(", ");
  Serial.println(constrain(target_speed + abs(regulator), minOut, maxOut));// Serial.print(", ");
}

void turnaround(float heading, float course){
  while(abs(heading - course) > 2.0){
    if (abs(course) < 1){
      // поворачиваем влево
      analogWrite(FRONT_LEFT, 0);      
      analogWrite(FRONT_RIGHT, turnaround_speed);
    }
    else if (abs(course)>179.0) {
      // поворачиваем вправо
      analogWrite(FRONT_LEFT, turnaround_speed);      
      analogWrite(FRONT_RIGHT, 0);
    }
    sensor.read();
    heading = sensor.axisZ;

    Serial.print(heading); Serial.print(", ");
    Serial.println(course);// Serial.print(", ");
  }
  // остановка, пауза, движение вперед
  analogWrite(FRONT_LEFT, 0);      
  analogWrite(FRONT_RIGHT, 0);
  unsigned long sensTime = millis();
  while (millis() < sensTime + 500) {}
  analogWrite(FRONT_LEFT, target_speed);      
  analogWrite(FRONT_RIGHT, target_speed);
  sensTime = millis();
  while (millis() < sensTime + 500) {}
  
}

// функция целеуказания
float get_course(unsigned long timer){

  float new_course;
  
  if ((timer / 5000) == 0) new_course = 0.0;
  else new_course = 180.0;
  Serial.print("nc=");Serial.print(new_course);

  return new_course;
}
