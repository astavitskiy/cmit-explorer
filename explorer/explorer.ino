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
 
// параметры регулятора
float kp = 15.0;
float ki = 0.0;
float kd = 0.0;
int dt = 10;
int minOut = 0;
int maxOut = 255;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  sensor.begin();
  sensor.setFastOffset();
}

void loop() {
  unsigned long sensTime = millis();

  if (sensTime - prevTime > dt) {
    sensor.read();
    float heading = sensor.axisZ;
    Serial.print(heading); Serial.print(", ");
    //Serial.println(sensTime);
    drive(heading, course);
    
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
void drive(float heading, float course){
  
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
