#include <SimpleFOC.h>

#define A_h 17 
#define A_l 16
#define B_h 35
#define B_l 36
#define C_h 6
#define C_l 5

const float pi = 3.141592;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_h, A_l, B_h, B_l, C_h, C_l);

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10);

float ma_per_degree = 10;
float detent_width = 20;
float last_current = 0.0;
float last_angle = 0.0;
int count = 0;
int i = 0;


// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&ma_per_degree, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void setup() { 
  
  pinMode(20, OUTPUT);

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

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);

  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // set the initial motor target
  motor.target = 0.2; // Amps - if phase resistance defined  

  // add target command T
  command.add('T', doTarget, "K Constant"); // - if phase resistance defined
  command.add('M', doMotor, "motor");
  //command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  i++;
  digitalWrite(20, HIGH);
  motor.loopFOC();
  float angle_rad = sensor.getAngle();
  float angle = angle_rad * (180/pi);
  float spring_current = 0;
  float total_current = 0;

  float threshold = detent_width / 2;
	float angle_noise_threshold = 0.3;

	if (angle > 0)
	{
		float phase = fmod(angle, detent_width);
		if (phase < threshold)
		{		
			if (last_current < 0)
			{
				if (abs(phase - threshold) > angle_noise_threshold)
				{
					spring_current = phase * (ma_per_degree / 1000.0);
				}
				else
				{
					spring_current = last_current;
				}
			}
			else
			{
				spring_current = phase * (ma_per_degree / 1000.0);
			}
		}
		else
		{
			if (last_current > 0)
			{
				if ((abs(phase - threshold)) > angle_noise_threshold)
				{
					float phase2 = fmod(phase, threshold);
					spring_current = (threshold - phase2) * (ma_per_degree / 1000.0) * -1;
				}
				else
				{
					spring_current = last_current;
				}
			}
			else
			{
				float phase2 = fmod(phase, threshold);
				spring_current = (threshold - phase2) * (ma_per_degree / 1000.0) * -1;
			}
		}
	}
	else
	{
		float phase = fmod(angle, detent_width);
		if (phase > -threshold)
		{
			if (last_current > 0)
			{
				if (abs(phase - -threshold) > angle_noise_threshold)
				{
					spring_current = phase * (ma_per_degree / 1000.0);
				}
				else
				{
					spring_current = last_current;
				}
			}
			else
			{
				spring_current = phase * (ma_per_degree / 1000.0);
			}
		}
		else
		{
			if (last_current < 0)
			{
				if ((abs(phase - -threshold)) > angle_noise_threshold)
				{
					float phase2 = fmod(phase, threshold);
					spring_current = (threshold + phase2) * (ma_per_degree / 1000.0);
				}
				else
				{
					spring_current = last_current;
				}
			}
			else
			{
				float phase2 = fmod(phase, threshold);
				spring_current = (threshold + phase2) * (ma_per_degree / 1000.0);
			}
		}
	}
	
	total_current = spring_current;
	if (total_current > 1.0)
		total_current = 1.0;
	else if (total_current < -1.0)
		total_current = -1.0;
	
	float peak_current_threshold = (ma_per_degree / 1000.0) * (detent_width / 2) * 0.9;
	bool at_snap_edge = abs(last_current) > peak_current_threshold && abs(total_current) > peak_current_threshold; // check that the transition that may be occuring is at the snap edge and not in the settling point between snaps
	if (last_current > 0 && total_current < 0 && at_snap_edge) // CCW snap
	{
		Serial.println(--count);		
	}
	else if(last_current < 0 && total_current > 0 && at_snap_edge) // CW snap
	{
		Serial.println(++count);
	}
	last_current = total_current;
	
	motor.move(total_current);
	
	command.run();
	digitalWrite(20, LOW);

  if(i % 100 == 0 && false)
  {
    String toprint = String(spring_current) + "\t" + String(total_current) + "\t" + String(angle);
    Serial.println(toprint);
  }
}