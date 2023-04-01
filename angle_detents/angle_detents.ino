#include <SimpleFOC.h>
#include <algorithm>

#define A_h 17 
#define A_l 16
#define B_h 36
#define B_l 35
#define C_h 6
#define C_l 5

const float pi = 3.141592;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(8);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_h, A_l, B_h, B_l, C_h, C_l);

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10);

float ma_per_degree = 85;
float damper_constant = 0;
float detent_width = 20;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&ma_per_degree, cmd); }
void doDamper(char* cmd) { command.scalar(&damper_constant, cmd);}
void doWidth(char* cmd) {command.scalar(&detent_width, cmd);}
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() { 
  
  pinMode(20, OUTPUT);

  // initialise magnetic sensor hardware
  sensor.init();
  sensor.min_elapsed_time = 0.005;
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 10;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.phase_resistance = 5;
  motor.current_limit = 1.0;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_VOLT_Q | _MON_VOLT_D | _MON_ANGLE | _MON_VEL;
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // set the initial motor target
  motor.target = 0.2; // Amps - if phase resistance defined  

  // add target command T
  command.add('T', doTarget, "K Constant"); // - if phase resistance defined
  command.add('D', doDamper, "Damping Constant");
  command.add('M', doMotor, "motor");
  command.add('W', doWidth, "Detent Width");
  //command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal:"));
  _delay(1000);
}

int i = 0;
void loop() {
  // main FOC algorithm function
  digitalWrite(20, HIGH);
  motor.loopFOC();
  
  //float cycle_time = 0.000300;
  float angle_rad = sensor.getAngle();
  //float velocity = sensor.getVelocity() * (180/pi);
  float angle = angle_rad * (180/pi);
  float spring_current = 0;
  float damper_current = 0;
  float total_current = 0;

  if(angle > 0)
  {
    float phase = fmod(angle, detent_width);
    if(phase < detent_width / 2)
    {
      spring_current = phase * (ma_per_degree / 1000.0);
    }
    else
    {
      float phase2 = fmod(phase, detent_width / 2.0);
      spring_current = ((detent_width / 2) - phase2) * (ma_per_degree / 1000.0) * -1;
    }
  }
  // else
  // {
  //   float phase = fmod(angle, detent_width);
  //   if(phase > (-detent_width/2))
  //   { 
  //     spring_current = phase * (ma_per_degree / 1000.0);
  //   }
  //   else
  //   {
  //     float phase2 = fmod(phase, detent_width / 2.0);
  //     spring_current = ((detent_width / 2) + phase2) * (ma_per_degree / 1000.0);
  //   }
  // }

  ///damper_current = velocity * damper_constant;
  total_current = spring_current;
  total_current = constrain(total_current, -1.0, 1.0);
  motor.move(total_current);

  //motor.monitor();
  if(i++ % 200 == 0){
    String toprint = String(total_current) + "\t" + String(angle) + "\t";
    Serial.println(toprint);
  }
  

  // user communication
  command.run();

  digitalWrite(20, LOW);
}