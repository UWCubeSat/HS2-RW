#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup()
{
  Serial.begin(115200);
  Serial.println("begin setup");

  if (bno.begin() == false)
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1){}
      ;
  }

  Serial.println("setup complete!");
}

void loop()
{
  sensors_event_t event;
  bno.getEvent(&event);

   //imu::Vector<3> acceleration = {event.acceleration.x, event.acceleration.y,event.acceleration.z};
  
  imu::Quaternion QuaternionData;
  QuaternionData= bno.getQuat();
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
   Serial.println(euler.z());
   
}
