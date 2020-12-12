#define BMX055_DISABLE_BMM

#define FRONT_LEFT 6
#define FRONT_RIGHT 10

#include <iarduino_Position_BMX055.h>
iarduino_Position_BMX055 sensor(BMX);

unsigned long prevTime = 4000; // задаем паузу перед началом движения
float course = 0.0; // задаем курс в градусах, по которому будем двигаться
 
// параметры регулятора
float kp = 0.0;
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
  return constrain(err * kp + integral + diff * kd, minOut, maxOut);
  
}

void drive(float heading, float course){
  
  int regulator = computePID(heading, course, kp, ki, kd, dt, minOut, maxOut);
  
  if (heading > course)
    {
      analogWrite(FRONT_LEFT, 200 - regulator);
      analogWrite(FRONT_RIGHT, 200 + regulator);
    }
    else if (heading < course)
    {
      analogWrite(FRONT_LEFT, 200 + regulator);
      analogWrite(FRONT_RIGHT, 200 - regulator);
    }
    else
    {
      analogWrite(FRONT_LEFT, 200);
      analogWrite(FRONT_RIGHT, 200);
    }
}
