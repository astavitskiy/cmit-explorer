#define BMX055_DISABLE_BMM

#define FRONT_LEFT 6
#define FRONT_RIGHT 10
#define PWM_MIN 0
#define PWM_MAX 255

#include <iarduino_Position_BMX055.h>
iarduino_Position_BMX055 sensor(BMX);

unsigned long prevTime = 4000; // задаем паузу перед началом движения
float course = 0.0; // задаем курс в градусах, по которому будем двигаться
int target_speed = 200; // задаем скорость движения
int minOut = 0;
int maxOut = 255;
 
// параметры регулятора
float kp = 15.0;
float ki = 0.5;
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

    // уточняем целевой курс
    check_course(sensTime);

    if (course == 180) {
      if (heading > 170) heading -= 180;
      if (heading < 0) heading += 180; 
    }

    Serial.print(sensTime); Serial.print(", ");
    Serial.print(heading); Serial.print(", ");
    Serial.print(course); Serial.print(", ");
    
    // едем прямо
    course_control(heading, course);
    
    prevTime = sensTime;    
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
  Serial.print(output); Serial.print(", ");
  //return constrain(output, minOut, maxOut);
  return output;
    
}

// функция езды по прямой
void course_control(float heading, float course){
  
  int regulator = computePID(heading, course, kp, ki, kd, dt, minOut, maxOut);
  
  if (heading > course) // подруливаем влево
    {
      analogWrite(FRONT_LEFT, constrain(target_speed - abs(regulator), minOut, maxOut));      
      analogWrite(FRONT_RIGHT, constrain(target_speed + abs(regulator), minOut, maxOut));
    }
    else if (heading < course) // подруливаем вправо
    {
      analogWrite(FRONT_LEFT, constrain(target_speed + abs(regulator), minOut, maxOut));
      analogWrite(FRONT_RIGHT, constrain(target_speed - abs(regulator), minOut, maxOut));
    }
    Serial.print(constrain(target_speed - abs(regulator), minOut, maxOut)); Serial.print(", ");
    Serial.println(constrain(target_speed + abs(regulator), minOut, maxOut));// Serial.print(", ");
}

// функция целеуказания
float check_course(unsigned long timer){
    
    if(abs(timer - 5000) < 100) {
      course = 90.0;
    }

    if(abs(timer - 10000) < 100) {
      course = 180.0;      
    }

    return course; 
}
