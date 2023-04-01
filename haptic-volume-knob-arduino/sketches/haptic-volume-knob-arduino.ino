#include "USB.h"
#include "USBHID.h"
#include <SimpleFOC.h>
#include <Adafruit_NeoPixel.h>
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

#define A_h 17 
#define A_l 16
#define B_h 35
#define B_l 36
#define C_h 6
#define C_l 5

#define LED_PIN 9
#define LED_COUNT 24

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

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
		HID.SendReport(1, data, 1);
		data[0] = 0x00;
		return HID.SendReport(1, data, 1);
	}
	
	bool decrement()
	{
		uint8_t data[1];
		data[0] = 0x02;
		HID.SendReport(1, data, 1);
		data[0] = 0x00;
		return HID.SendReport(1, data, 1);
	}

	bool send(uint8_t * value) {
		return HID.SendReport(0, value, 8);
	}
};

QueueHandle_t cmdQueueHandle = NULL;
QueueHandle_t percentQueueHandle = NULL;
QueueHandle_t count_queue_handle = NULL;
TaskHandle_t led_task_handle = NULL;
TaskHandle_t queue_task_handle = NULL;
TaskHandle_t count_task_handle = NULL;

CustomHIDDevice Device;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_h, A_l, B_h, B_l, C_h, C_l);

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10);


constexpr int detents_per_revolution = 50; // doesn't work reliably after 50
constexpr float detent_width = 360.0f / detents_per_revolution;
constexpr float detent_max_ma = 600;
constexpr float ma_per_degree = detent_max_ma / detent_width;


//float ma_per_degree = 45; // K constant
//float detent_width = 7.2; // Width between detents in angles
float last_current = 0.0;
float last_angle = 0.0;
int count = 0;
int i = 0;

Commander command = Commander(Serial); // instantiate the commander
//void doTarget(char* cmd) { command.scalar(&ma_per_degree, cmd); }
//void doWidth(char* cmd) {command.scalar(&detent_width, cmd); }
//void doMotor(char* cmd) { command.motor(&motor, cmd); }
TaskHandle_t motor_task_handle = NULL;

void setup_motor()
{
	pinMode(20, OUTPUT);
	sensor.init(); // initialise magnetic sensor hardware
	motor.linkSensor(&sensor); // link the motor to the sensor

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
	
	motor.init();
	motor.initFOC(); // align sensor and start FOC

	//command.add('T', doTarget, "K Constant"); // - if phase resistance defined
	//command.add('M', doMotor, "motor");
	//command.add('W', doWidth, "Detent Width");

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target using serial terminal:"));
	vTaskDelay(pdMS_TO_TICKS(1000));
}


//void queue_task(void* arg)
//{
//	bool recv;
//	while (1)
//	{
//		if (xQueueReceive(cmdQueueHandle, &recv, TickType_t(5)))
//		{
//			if (recv)
//				Device.increment();
//			else
//				Device.decrement();
//			
//			Serial.println("Sending update: " + String(recv));
//		}
//		vTaskDelay(10);
//	}
//}

void count_update_task(void* arg)
{
	int recv_count;
	while (1)
	{
		if(xQueueReceive(count_queue_handle, &recv_count, (TickType_t)5))
		{
			uint32_t count_constrained = constrain(recv_count, 0, detents_per_revolution);
			float levels_per_led = (float)detents_per_revolution / (float)LED_COUNT;
			int fully_lit_count = (int)(count_constrained / levels_per_led);
			float remainder = fmod(count_constrained, levels_per_led);
			float last_led_percentage = remainder / levels_per_led;
			set_strip(fully_lit_count, last_led_percentage);
		}
	}
}

void set_strip(int fully_lit, float last_percent)
{
	String msg = "Setting " + String(fully_lit) + " fully lit LEDs";
	Serial.println(msg);
	msg = "Last LED percent: " + String(last_percent, 2);
	Serial.println(msg);
	
	strip.clear();
	for (int j = 0; j < fully_lit; j++)
	{
		strip.setPixelColor(j, 0, 255, 0);
	}
	
	if (last_percent > 0.001 && fully_lit < LED_COUNT) 
	{
		int brightness = (int)(last_percent * 255);
		int last_led_idx = fully_lit; 
		strip.setPixelColor(last_led_idx, 0, brightness, 0);
	}
	
	strip.show();
}

//void set_strip(int percent)
//{
//	strip.clear();
//	int num_green = round((float(percent) / 100.0f) * 24);
//	Serial.println("Setting " + String(num_green) + "LEDs green");
//	for (int i = 0; i < num_green; i++)
//	{
//		strip.setPixelColor(i, 0, 150, 0);	
//	}
//	strip.show();
//}

void set_strip_single_color(uint8_t red, uint8_t green, uint8_t blue)
{
	for (int i = 0; i < LED_COUNT; i++)
	{
		strip.setPixelColor(i, red, green, blue);
	}
	strip.show();
}

//void led_task(void* arg)
//{
//	uint8_t buffer;
//	while (1)
//	{
//		if(xQueueReceive(percentQueueHandle, &buffer, (TickType_t)10))
//		{
//			uint8_t test_int = buffer;
//			//Serial.println("Percentage: " + String(buffer));
//			set_strip(test_int);
//		}
//		vTaskDelay(10);
//	}
//	while (1)
//	{
//		strip.clear();
//		for (int i = 0; i < strip.numPixels(); i++)
//		{
//			strip.setPixelColor(i, 0, 0, 50);
//			strip.show();
//		}
//		
//	}
//}


void setup()
{
	strip.begin();
	strip.show();
	strip.clear();
	strip.setBrightness(35);
	set_strip_single_color(50, 0, 0);
	Device.begin();
	USB.begin();
	
	
	//cmdQueueHandle = xQueueCreate(5, sizeof(bool));
	//percentQueueHandle = xQueueCreate(5, sizeof(uint8_t));
	count_queue_handle = xQueueCreate(5, sizeof(int));
	
	//xTaskCreate(queue_task, "Queue Task", 2048, NULL, configMAX_PRIORITIES - 5, &queue_task_handle);
	//xTaskCreate(led_task, "LED Task", 2048, NULL, configMAX_PRIORITIES - 7, &led_task_handle);
	xTaskCreate(count_update_task, "Count Task", 2048, NULL, configMAX_PRIORITIES - 5, &count_task_handle);
	
	setup_motor();
	pinMode(LED_BUILTIN, OUTPUT);
	set_strip_single_color(0, 0, 50);
}

void loop()
{
	digitalWrite(20, HIGH);
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
	
//	if (i++ % 1 == 0)
//	{
//		Serial.println(String(spring_current) + "\t" + String(angle));
//	}
	
	total_current = spring_current;
	
	if (total_current > 1.0)
		total_current = 1.0;
	else if (total_current < -1.0)
		total_current = -1.0;
	
	float peak_current_threshold = (ma_per_degree / 1000.0) * (detent_width / 2) * 0.9;
	bool at_snap_edge = abs(last_current) > peak_current_threshold && abs(total_current) > peak_current_threshold; // check that the transition that may be occuring is at the snap edge and not in the settling point between snaps
	//bool set_led = false;
	if (last_current > 0 && total_current < 0 && at_snap_edge) // CCW snap
	{
		Serial.println(--count);
		//bool change = false;
		//xQueueSend(cmdQueueHandle, &change, 5);		
		xQueueSend(count_queue_handle, &count, (TickType_t)5);
		//set_led = true;
	}
	else if(last_current < 0 && total_current > 0 && at_snap_edge) // CW snap
	{
		Serial.println(++count);
		//bool change = true;
		//xQueueSend(cmdQueueHandle, &change, 5);
		int temp_int = count;
		xQueueSend(count_queue_handle, &temp_int, (TickType_t)5);
		//set_led = true;
	}
	
//	if (set_led)
//	{
//		uint8_t count_limited = (uint8_t)constrain(count, 0, 100);
//		xQueueSend(percentQueueHandle, &count_limited, (TickType_t)5);	
//	}	
	
	last_current = total_current;
	
	motor.move(total_current);
	
	digitalWrite(20, LOW);
}