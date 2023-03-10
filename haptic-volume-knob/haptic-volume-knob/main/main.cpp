#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "Arduino.h"

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

float ma_per_degree = 85; // K constant
float detent_width = 10; // Width between detents in angles
float last_current = 0.0;
float last_angle = 0.0;
int count = 0;

Commander command = Commander(Serial); // instantiate the commander
void doTarget(char* cmd) { command.scalar(&ma_per_degree, cmd); }
void doWidth(char* cmd) {command.scalar(&detent_width, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }
int i = 0;
TaskHandle_t motor_task_handle = NULL;

void setup_motor()
{
	gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
	sensor.init(); // initialise magnetic sensor hardware
	sensor.min_elapsed_time = 0.005;
	motor.linkSensor(&sensor); 	// link the motor to the sensor

	// driver config
	driver.voltage_power_supply = 10; // power supply voltage [V]
	driver.init();
	
	// motor config
	motor.linkDriver(&driver); // link driver
	motor.phase_resistance = 5; // this sets align voltage too
	motor.current_limit = 1.0;
	motor.torque_controller = TorqueControlType::voltage;
	motor.controller = MotionControlType::torque; // set motion control loop to be used
	
	Serial.begin(115200); // use monitoring with serial 
	motor.useMonitoring(Serial);
	motor.monitor_variables = _MON_ANGLE | _MON_VEL;
	
	motor.init();
	motor.initFOC(); // align sensor and start FOC

	command.add('T', doTarget, "K Constant"); // - if phase resistance defined
	command.add('M', doMotor, "motor");
	command.add('W', doWidth, "Detent Width");

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target using serial terminal:"));
	_delay(1000);
}

void run_loop_detent()
{	
	// main FOC algorithm function
	gpio_set_level(GPIO_NUM_27, 1);
	motor.loopFOC();
  
	float angle_rad = sensor.getAngle();
	float angle = angle_rad * (180 / PI);
	float spring_current = 0;
	float total_current = 0;

	if (angle > 0)
	{
		float phase = fmod(angle, detent_width);
		if (phase < detent_width / 2)
		{
			if (last_current < 0)
			{
				if (abs(phase - detent_width / 2) > 0.1)
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
				if ((abs(phase - detent_width / 2)) > 0.1)
				{
					float phase2 = fmod(phase, detent_width / 2);
					spring_current = (detent_width / 2 - phase2) * (ma_per_degree / 1000.0) * -1;
				}
				else
				{
					spring_current = last_current;
				}
			}
			else
			{
				float phase2 = fmod(phase, detent_width / 2);
				spring_current = (detent_width / 2 - phase2) * (ma_per_degree / 1000.0) * -1;
			}
		}
	}
	
	total_current = spring_current;
	total_current = constrain(total_current, -1.0, 1.0);
	
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
	
	//Serial.println(angle);
	command.run();
	gpio_set_level(GPIO_NUM_27, 0);
}

void motor_task(void* arg)
{
	setup_motor();
	while (1)
	{
		run_loop_detent();	
	}
}

extern "C" void app_main(void)
{
	disableCore1WDT();
	xTaskCreatePinnedToCore(motor_task, "Motor Task", 4096, NULL, configMAX_PRIORITIES - 1, &motor_task_handle, 1);
	while (true)
	{
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}