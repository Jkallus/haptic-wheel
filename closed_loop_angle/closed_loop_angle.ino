#include <SimpleFOC.h>

#define A_h 17 
#define A_l 16
#define B_h 26
#define B_l 25
#define C_h 22
#define C_l 21

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_h, A_l, B_h, B_l, C_h, C_l);

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 5);

float target_angle = 1;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void setup() { 
  
  // initialise magnetic sensor hardware
  sensor.init();
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
  motor.velocity_limit = 10;
  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
  //motor.torque_controller = TorqueControlType::voltage;
  
  // velocity PI controller parameters
  motor.PID_velocity.P = 0.07;
  motor.PID_velocity.I = 5.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01f;

 // angle P controller
  motor.P_angle.P = 20;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // set the initial motor target
  //motor.target = 0.2; // Amps - if phase resistance defined  
  //motor.target = 2; // Volts 

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('M', doMotor, "motor");
  //command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal:"));
  _delay(1000);
}



void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move(target_angle);

  //motor.monitor();

  // user communication
  command.run();
}