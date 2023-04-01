#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp32-hal.h"
#include "Arduino.h"

#include <SimpleFOC.h>
#include  <hal/wdt_hal.h>
#include <esp_task_wdt.h>

#include "USB.h"
#include "USBHID.h"


#define A_h 17 
#define A_l 16
#define B_h 35
#define B_l 36
#define C_h 6
#define C_l 5


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_h, A_l, B_h, B_l, C_h, C_l);

USBHID HID;

static const uint8_t report_descriptor[] = { // 8 axis
    0x05, 0x0C,    // UsagePage(Consumer[12])
    0x09, 0x01,    // UsageId(Consumer Control[1])
    0xA1, 0x01,    // Collection(Application)
    0x85, 0x01,    //     ReportId(1)
    0x09, 0xE9,    //     UsageId(Volume Increment[233])
    0x15, 0x00,    //     LogicalMinimum(0)
    0x25, 0x01,    //     LogicalMaximum(1)
    0x95, 0x01,    //     ReportCount(1)
    0x75, 0x01,    //     ReportSize(1)
    0x81, 0x02,    //     Input(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, BitField)
    0x09, 0xEA,    //     UsageId(Volume Decrement[234])
    0x81, 0x02,    //     Input(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, BitField)
    0x75, 0x06,    //     ReportSize(6)
    0x81, 0x03,    //     Input(Constant, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, BitField)
    0x09, 0xE0,    //     UsageId(Volume[224])
    0x15, 0xFF,    //     LogicalMinimum(-1)
    0x75, 0x02,    //     ReportSize(2)
    0x91, 0x02,    //     Output(Data, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0x75, 0x06,    //     ReportSize(6)
    0x91, 0x03,    //     Output(Constant, Variable, Absolute, NoWrap, Linear, PreferredState, NoNullPosition, NonVolatile, BitField)
    0xC0,          // EndCollection()
};

QueueHandle_t cmdQueueHandle = NULL;
TaskHandle_t queue_task_handle = NULL;

class CustomHIDDevice : public USBHIDDevice {
public:
	CustomHIDDevice(void) {
		static bool initialized = false;
		if (!initialized) {
			initialized = true;
			HID.addDevice(this, sizeof(report_descriptor));
		}
	}
  
	void begin(void) {
		HID.begin();
	}

	void _onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len) override
	{
		Serial.println("Received report");
		Serial.println("Report ID: " + String(report_id));
		Serial.println("Report Length:" + String(len));
		for (int i = 0; i < len; i++) {
			Serial.print(buffer[i], HEX);
		}
		Serial.println();
	}
    
	uint16_t _onGetDescriptor(uint8_t* buffer) {
		memcpy(buffer, report_descriptor, sizeof(report_descriptor));
		return sizeof(report_descriptor);
	}

	bool increment()
	{
		uint8_t data[1];
		data[0] = 0x01;
		return HID.SendReport(1, data, 1);
	}
	
	bool decrement()
	{
		uint8_t data[1];
		data[0] = 0x02;
		return HID.SendReport(1, data, 1);
	}

	bool send(uint8_t * value) {
		return HID.SendReport(0, value, 8);
	}
};

CustomHIDDevice Device;

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10);

float ma_per_degree = 10; // K constant
float detent_width = 20; // Width between detents in angles
float last_current = 0.0;
float last_angle = 0.0;
int count = 0;
int i = 0;

Commander command = Commander(Serial); // instantiate the commander
void doTarget(char* cmd) { command.scalar(&ma_per_degree, cmd); }
void doWidth(char* cmd) {command.scalar(&detent_width, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }
TaskHandle_t motor_task_handle = NULL;

void setup_motor()
{
	gpio_set_direction(GPIO_NUM_20, GPIO_MODE_OUTPUT);
	sensor.init(); // initialise magnetic sensor hardware
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
	//motor.useMonitoring(Serial);
	//motor.monitor_variables = _MON_ANGLE | _MON_VEL;
	
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
{	i++;
	// main FOC algorithm function
	if (i % 1000 == 0)
	{
		gpio_set_level(GPIO_NUM_20, 1);			
	}
	
	motor.loopFOC();
  
	float angle_rad = sensor.getAngle();
	float angle = angle_rad * (180 / PI);
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
	if (i % 1000 == 0)
	{
		gpio_set_level(GPIO_NUM_20, 0);	
	}
	
}

void motor_task(void* arg)
{
	int q = 0;
	setup_motor();
	while (1)
	{
		run_loop_detent();
		esp_task_wdt_reset();	
//		if (q++ % 1000 == 0)
//		{
//			esp_task_wdt_reset();	
//		}
	}
}

void queue_task(void* arg)
{
	bool recv;
	while (1)
	{
		if(xQueueReceive(cmdQueueHandle, &recv, TickType_t(5)))
		{
			if (recv)
				Device.increment();
			else
				Device.decrement();
		}
	}
}

extern "C" void app_main(void)
{
	cmdQueueHandle = xQueueCreate(10, sizeof(bool));
	
	Device.begin();
	USB.begin();
	
	xTaskCreatePinnedToCore(motor_task, "Motor Task", 4096, NULL, configMAX_PRIORITIES - 1, &motor_task_handle, 1);
	xTaskCreatePinnedToCore(queue_task, "Queue Task", 1024, NULL, configMAX_PRIORITIES - 2, &queue_task_handle, 0);
	
	while (true)
	{
		vTaskDelay(1);
	}
}