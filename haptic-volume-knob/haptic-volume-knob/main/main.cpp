#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "Arduino.h"

#include <SimpleFOC.h>

static const char *TAG = "example";

#define A_h 17 
#define A_l 16
#define B_h 26
#define B_l 25
#define C_h 22
#define C_l 21

const float pi = 3.141592;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_h, A_l, B_h, B_l, C_h, C_l);

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 5);

float ma_per_degree = 10;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&ma_per_degree, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }

int i = 0;

TaskHandle_t motor_task_handle = NULL;

void setup_motor()
{
	pinMode(27, OUTPUT);
	
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
	command.add('M', doMotor, "motor");
	//command.add('T', doTarget, "target voltage");

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target using serial terminal:"));
	_delay(1000);
}

void run_loop()
{
	// main FOC algorithm function
	i++;
	digitalWrite(27, HIGH);
	motor.loopFOC();
	float cycle_time = 0.0002522;
	float angle_rad = sensor.getAngle();
	float velocity = sensor.getVelocity() * (180 / pi);
	float angle = angle_rad * (180 / pi);
	float threshold = 180;
	float spring_current = 0;
	float damper_current = 0;

	if (angle < threshold)
	{
		motor.move(0);
	}
	else
	{
		spring_current = (angle - threshold) * (ma_per_degree / 1000.0);
		damper_current = velocity * ((ma_per_degree / 1000.0) * cycle_time) / 2;
		float total_current = spring_current; // + damper_current;
		if (total_current > 1.0)
		{
			total_current = 1.0;
		}
		motor.move(total_current);
	}

	if (i % 100 == 0  && false)
	{
		String toprint = String(spring_current) + "\t" + String(damper_current) + "\t" + String(velocity) + "\t" + String(angle) + "\n";
		Serial.print(toprint);
	}
  
  
	//motor.monitor();

	// user communication
	command.run();
	digitalWrite(27, LOW);
}

void motor_task(void* arg)
{
	setup_motor();
	while (1)
	{
		run_loop();	
	}
}

extern "C" void app_main(void)
{
	xTaskCreatePinnedToCore(motor_task, "Motor Task", 4096, NULL, configMAX_PRIORITIES - 1, &motor_task_handle, 1);
	while (true)
	{
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
