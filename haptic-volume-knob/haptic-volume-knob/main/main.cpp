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

float ma_per_degree = 85;
float damper_constant = 0;
float detent_width = 7.2;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&ma_per_degree, cmd); }
void doDamper(char* cmd) { command.scalar(&damper_constant, cmd); }
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
	motor.monitor_variables = _MON_VOLT_Q | _MON_VOLT_D | _MON_ANGLE | _MON_VEL;
	
	motor.init();
	motor.initFOC(); // align sensor and start FOC

	command.add('T', doTarget, "K Constant"); // - if phase resistance defined
	command.add('D', doDamper, "Damping Constant");
	command.add('M', doMotor, "motor");
	command.add('W', doWidth, "Detent Width");

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target using serial terminal:"));
	_delay(1000);
}

void run_loop()
{
	// main FOC algorithm function
	gpio_set_level(GPIO_NUM_27, 1);
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
		//damper_current = velocity * ((ma_per_degree / 1000.0) * cycle_time) / 2;
		float total_current = spring_current; // + damper_current;
		if (total_current > 1.0)
		{
			total_current = 1.0;
		}
		motor.move(total_current);
	}  
  
	//motor.monitor();

	// user communication
	command.run();
	gpio_set_level(GPIO_NUM_27, 0);
}

void run_loop_detent()
{	
	// main FOC algorithm function
	gpio_set_level(GPIO_NUM_27, 1);
	motor.loopFOC();
  
	float cycle_time = 0.000300;
	float angle_rad = sensor.getAngle();
	float velocity = sensor.getVelocity() * (180 / pi);
	float angle = angle_rad * (180 / pi);
	float spring_current = 0;
	float damper_current = 0;
	float total_current = 0;

	if (angle > 0)
	{
		float phase = fmod(angle, detent_width);
		if (phase < detent_width / 2)
		{
			spring_current = phase * (ma_per_degree / 1000.0);
		}
		else
		{
			float phase2 = fmod(phase, detent_width / 2.0);
			spring_current = ((detent_width / 2) - phase2) * (ma_per_degree / 1000.0) * -1;
		}
	}
	else
	{
		float phase = fmod(angle, detent_width);
		if (phase > (-detent_width / 2))
		{ 
			spring_current = phase * (ma_per_degree / 1000.0);
		}
		else
		{
			float phase2 = fmod(phase, detent_width / 2.0);
			spring_current = ((detent_width / 2) + phase2) * (ma_per_degree / 1000.0);
		}
	}

	///damper_current = velocity * damper_constant;
	total_current = spring_current;
	total_current = constrain(total_current, -1.0, 1.0);
	motor.move(total_current);

//	motor.monitor();
//	if (i++ % 200 == 0) {
//		String toprint = String(spring_current) + "\t" + String(damper_current) + "\t"  + String(angle) + "\t" + String(velocity) +  "\n";
//		Serial.print(toprint);
//	}
  
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