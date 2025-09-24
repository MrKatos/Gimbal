#include <Arduino.h>
// #include <SimpleFOC.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Wire.h>
#include <ArduinoEigen.h>

using namespace Eigen;

// angle set point variable
float target_angle_pitch = 0;
float target_angle_yaw = 0;
float target_angle_roll  = 0;

// gyro instance
BNO08x BNO085;

#define BNO08X_CS   25
#define BNO08X_INT  33
#define BNO08X_RST  32

// // motors instance
// BLDCMotor pitch_motor = BLDCMotor(7, 15.2, 21);
// BLDCMotor yaw_motor = BLDCMotor(7, 15.2, 21);
// BLDCMotor roll_motor = BLDCMotor(7, 15.2, 21);

// // driver instance
// BLDCDriver3PWM pitch_driver = BLDCDriver3PWM(15, 2, 4, 16);
// BLDCDriver3PWM yaw_driver = BLDCDriver3PWM(17, 5, 21, 3);
// BLDCDriver3PWM roll_driver = BLDCDriver3PWM(1, 22, 13, 12);

// // sensor instance
// MagneticSensorSPI pitch_sensor = MagneticSensorSPI(14, 14, 0x3FFF);
// MagneticSensorSPI yaw_sensor = MagneticSensorSPI(27, 14, 0x3FFF);
// MagneticSensorSPI roll_sensor = MagneticSensorSPI(26, 14, 0x3FFF);

void setReports(void) {
  Serial.println("Setting desired reports");
  if (BNO085.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
  delay(100);
}

float sinD(float deg) {
  return sin(deg * PI / 180.0);
}

float cosD(float deg) {
  return cos(deg * PI / 180.0);
}

float radToDeg(float rad) {
  return rad * 180.0 / PI;
}

struct Quat {
  float w, x, y, z;
};

Quat quatNormalize(Quat q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n == 0) n = 1;
  q.w /= n; q.x /= n; q.y /= n; q.z /= n;
  return q;
}

Quat quatInverse(Quat q) {
  return { q.w, -q.x, -q.y, -q.z };
}

Quat quatMultiply(Quat a, Quat b) {
  Quat r;
  r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  return r;
}

struct CompensatingAngles {
  float roll;
  float pitch;
  float yaw;
};

CompensatingAngles getCompensatingAngles(Quat q) {
    q = quatNormalize(q);
    Quat q_inv = quatInverse(q);
    
    // Roll
    float sinr_cosp = 2 * (q_inv.w * q_inv.x + q_inv.y * q_inv.z);
    float cosr_cosp = 1 - 2 * (q_inv.x * q_inv.x + q_inv.y * q_inv.y);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch
    float sinp = 2 * (q_inv.w * q_inv.y - q_inv.z * q_inv.x);
    float pitch;
    if (abs(sinp) >= 1)
        pitch = copysignf(M_PI / 2, sinp); 
    else
        pitch = asinf(sinp);

    // Yaw 
    float siny_cosp = 2 * (q_inv.w * q_inv.z + q_inv.x * q_inv.y);
    float cosy_cosp = 1 - 2 * (q_inv.y * q_inv.y + q_inv.z * q_inv.z);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    CompensatingAngles result;
    result.roll = radToDeg(roll);
    result.pitch = radToDeg(pitch);
    result.yaw = radToDeg(yaw);
    
    return result;
}

// --- TASK 1 SENSOR READING & CALCUTATIONS---
void Task_BMO(void *parameter){
  while (true)
  {
    // gyro
    if (BNO085.wasReset()) {
      Serial.print("sensor was reset ");
      setReports();
    }
  
    if (BNO085.getSensorEvent() == true) {
      if (BNO085.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
        
        // float Y = (BNO085.getPitch()) * 180.0 / PI;
        // float Z = (BNO085.getYaw()) * 180.0 / PI;
        // float X = (BNO085.getRoll()) * 180.0 / PI;

        float quatI = BNO085.getQuatI();
        float quatJ = BNO085.getQuatJ();
        float quatK = BNO085.getQuatK();
        float quatReal = BNO085.getQuatReal();
        // float quatRadianAccuracy = BNO085.getQuatRadianAccuracy();

        Quat current_q = {quatReal, quatI, quatJ, quatK};
        CompensatingAngles ik = getCompensatingAngles(current_q);
        Serial.print(quatReal, 3);
        Serial.print(F(","));
        Serial.print(quatI, 3);
        Serial.print(F(","));
        Serial.print(quatJ, 3);
        Serial.print(F(","));
        Serial.print(quatK, 3);
        Serial.print(F(","));
        Serial.print(ik.yaw, 3);
        Serial.print(F(","));
        Serial.print(ik.pitch, 3);
        Serial.print(F(","));
        Serial.print(ik.roll, 3);
        Serial.println();
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  } 
}

// // --- TASK 2 MOTORS---
// void Task_Motor_Control(void *parameter){
//   while (true)
//   {
//     // main FOC algorithm function
//     pitch_motor.loopFOC();
//     // Motion control function
//     pitch_motor.move(target_angle_pitch);

//     // main FOC algorithm function
//     yaw_motor.loopFOC();
//     // Motion control function
//     yaw_motor.move(target_angle_yaw);

//     // main FOC algorithm function
//     roll_motor.loopFOC();
//     // Motion control function
//     roll_motor.move(target_angle_roll);
//   }
// }

void setup() {
  // // initialize magnetic encoder sensor
  // pitch_sensor.init();
  // yaw_sensor.init();
  // roll_sensor.init();

  // // link motor to the sensor
  // pitch_motor.linkSensor(&pitch_sensor);
  // yaw_motor.linkSensor(&yaw_sensor);
  // roll_motor.linkSensor(&roll_sensor);

  // // initialize motor driver
  // pitch_driver.init();
  // yaw_driver.init();
  // roll_driver.init();

  // // link driver to the motor
  // pitch_motor.linkDriver(&pitch_driver);
  // yaw_motor.linkDriver(&yaw_driver);
  // roll_motor.linkDriver(&roll_driver);

  // // set motion control loop to be used
  // pitch_motor.controller = MotionControlType::angle;
  // yaw_motor.controller = MotionControlType::angle;
  // roll_motor.controller = MotionControlType::angle;

  // // controller configuration 
  // // default parameters in defaults.h

  // // controller configuration based on the control type 
  // // velocity PID controller parameters
  // // default P=0.5 I = 10 D =0
  // pitch_motor.PID_velocity.P = 0.2;
  // pitch_motor.PID_velocity.I = 20;
  // pitch_motor.PID_velocity.D = 0.001;

  // yaw_motor.PID_velocity.P = 0.2;
  // yaw_motor.PID_velocity.I = 20;
  // yaw_motor.PID_velocity.D = 0.001;

  // roll_motor.PID_velocity.P = 0.2;
  // roll_motor.PID_velocity.I = 20;
  // roll_motor.PID_velocity.D = 0.001;

  // // jerk control using voltage voltage ramp
  // // default value is 300 volts per sec  ~ 0.3V per millisecond
  // pitch_motor.PID_velocity.output_ramp = 1000;
  // yaw_motor.PID_velocity.output_ramp = 1000;
  // roll_motor.PID_velocity.output_ramp = 1000;

  // // velocity low pass filtering
  // // default 5ms - try different values to see what is the best. 
  // // the lower the less filtered
  // pitch_motor.LPF_velocity.Tf = 0.01;
  // yaw_motor.LPF_velocity.Tf = 0.01;
  // roll_motor.LPF_velocity.Tf = 0.01;

  // // angle P controller -  default P=20
  // pitch_motor.P_angle.P = 20;
  // yaw_motor.P_angle.P = 20;
  // roll_motor.P_angle.P = 20;

  // //  maximal velocity of the position control
  // // default 20
  // pitch_motor.velocity_limit = 4;
  // yaw_motor.velocity_limit = 4;
  // roll_motor.velocity_limit = 4;
  // // default voltage_power_supply
  // pitch_motor.voltage_limit = 12;
  // yaw_motor.voltage_limit = 12;
  // roll_motor.voltage_limit = 12;

  // // use monitoring with serial 
  Serial.begin(115200);
  // // comment out if not needed
  // pitch_motor.useMonitoring(Serial);
  // roll_motor.useMonitoring(Serial);
  // yaw_motor.useMonitoring(Serial);
  
  // // initialize motor
  // pitch_motor.init();
  // roll_motor.init();
  // yaw_motor.init();
  // // align encoder and start FOC
  // pitch_motor.initFOC();
  // roll_motor.initFOC();
  // yaw_motor.initFOC();


  // Serial.println("Motor ready.");
  // _delay(1000);

  while(!Serial) delay(10); // wait for serial port to connect
  if (BNO085.beginSPI(BNO08X_CS, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  setReports();

  Serial.println("Reading events");
  delay(100);

  xTaskCreatePinnedToCore(Task_BMO, "Gyro", 8192, NULL, 1, NULL, 0);
}

void loop() {
  // // main FOC algorithm function
  // pitch_motor.loopFOC();
  // // Motion control function
  // pitch_motor.move(target_angle_pitch);

  // // main FOC algorithm function
  // yaw_motor.loopFOC();
  // // Motion control function
  // yaw_motor.move(target_angle_yaw);

  // // main FOC algorithm function
  // roll_motor.loopFOC();
  // // Motion control function
  // roll_motor.move(target_angle_roll);

  // // gyro
  // if (BNO085.wasReset()) {
  //   Serial.print("sensor was reset ");
  //   setReports();
  // }
 
  // // Has a new event come in on the Sensor Hub Bus?
  // if (BNO085.getSensorEvent() == true) {

  //   // is it the correct sensor data we want?
  //   if (BNO085.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

  //     float pitch = (BNO085.getPitch()) * 180.0 / PI;
  //     float yaw = (BNO085.getYaw()) * 180.0 / PI;
  //     float roll = (BNO085.getRoll()) * 180.0 / PI;
      
  //     float quatReal = BNO085.getQuatReal();
  //     float quatRadianAccuracy = BNO085.getQuatRadianAccuracy();

  //     Serial.print(roll, 1);
  //     Serial.print(F(","));
  //     Serial.print(pitch, 1);
  //     Serial.print(F(","));
  //     Serial.print(yaw, 1);

  //     Serial.println();
  //   }
  // }
}
