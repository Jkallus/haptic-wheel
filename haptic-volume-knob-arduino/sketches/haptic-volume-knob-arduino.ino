#include "USB.h"
#include "USBHID.h"
#include <SimpleFOC.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <BH1750.h>

#ifdef LOG_LOCAL_LEVEL
	#undef LOG_LOCAL_LEVEL
#endif

#define LOG_LOCAL_LEVEL ESP_LOG_WARN

////////////////// HID Config ////////////////////
const char* HID_TAG = "HID";
const char* MOTOR_TAG = "MOTOR";
const char* VOLUME_TAG = "VOLUME";
const char* MAIN_TAG = "MAIN";
const char* LED_TAG = "LED";
const char* BUTTON_TAG = "BUTTON";
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
USBHID HID;

////////////////// Motor Config //////////////////
#define A_h 17 
#define A_l 16
#define B_h 35
#define B_l 36
#define C_h 6
#define C_l 5

enum class DETENTS_PER_REV
{
	Fifty      = 50,
	TwentyFive = 25,
};

constexpr DETENTS_PER_REV detents_per_revolution = DETENTS_PER_REV::Fifty;
constexpr float detent_width = 360.0f / (float)detents_per_revolution;
float detent_max_ma = 500;
float detent_max_ma_values[] = { 200, 300, 400, 500, 600, 700 };
SemaphoreHandle_t detent_ma_semaphore = NULL;
constexpr float virtual_wall_ma_per_degree = 350;
float ma_per_degree = detent_max_ma / detent_width;

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_h, A_l, B_h, B_l, C_h, C_l);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10);

float init_angle = 0.0;
float max_angle = 0.0;
float last_current = 0.0;
float last_angle = 0.0;
float max_angle_threshold = 0.0; // angle threshold for CW after count = 50 (likely negative number)
float min_angle_threshold = 0.0; // angle threshold for CCW after count = 0 (likely positive number)
int count = 0; // number of detents were at out of max, 0...50 or 0...25. 
int i = 0; // loop variable
Commander command = Commander(Serial); // instantiate the commander
QueueHandle_t count_queue_handle = NULL;
TaskHandle_t count_change_internal_handle = NULL;

/////////////////////// Button Config //////////////////////////
gpio_num_t button_pin = GPIO_NUM_38;
QueueHandle_t button_message_queue = NULL;
TaskHandle_t button_task_handle = NULL;
unsigned long button_time = 0;
unsigned long last_button_time = 0;
uint32_t button_presses = 0;
void IRAM_ATTR button_isr()
{
	button_time = millis();
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if (button_time - last_button_time > 350)
	{
		button_presses++;
		last_button_time = button_time;
		xQueueSendFromISR(button_message_queue, &button_presses, &xHigherPriorityTaskWoken);
		
		if (xHigherPriorityTaskWoken)
		{
			portYIELD_FROM_ISR();
		}
	}
}

//////////////// LED Config ////////////////////////
#define LED_PIN 9
#define LED_COUNT 24

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
QueueHandle_t led_state_queue_handle = NULL;
TaskHandle_t led_task_handle = NULL;

int led_order[24] = {7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 0, 1, 2, 3, 4, 5, 6};

//////////////// Light Sensor Config ///////////////////

#define SDA_PIN 7
#define SCL_PIN 15

TaskHandle_t light_monitor_task_handle;

BH1750 lightMeter;

void setup_light_sensor()
{
	ESP_LOGI(LED_TAG, "Setting up light sensor");
	Wire.setPins(SDA_PIN, SCL_PIN);
	Wire.begin();
	lightMeter.begin();
	
}

float get_light_level()
{
	return lightMeter.readLightLevel();
}

int calculate_brightness_for_lux(float lux)
{
	if (lux < 5) // Just computer monitor brightness
	{
		return 10;
	}
	else if (lux < 30) // Just one lamp far away
	{
		return 15;
	}
	else if (lux < 500) // Just one lamp close by
	{
		return 30;
	}
	else if(lux < 3000)
	{
		return 50;
	}
	else if(lux < 10000)
	{
		return 80;
	}
	else
	{
		return 255;
	}
}

int get_best_led_level()
{
	float val = get_light_level();
	int setting = calculate_brightness_for_lux(val);
	//ESP_LOGI(LED_TAG, "Measured %f lux, setting LED brightness to %d", val, setting);
	return setting;
}

void light_monitor_task(void* arg)
{
	while (1)
	{
		strip.setBrightness(get_best_led_level());
		vTaskDelay(pdMS_TO_TICKS(1000 * 5)); // update brightness every 5 seconds
	}
}

uint8_t ConvertVolumeLevelToCount(uint8_t volume_level) // returns the count associated with a volume level
{
	if (detents_per_revolution == DETENTS_PER_REV::Fifty)
	{
		return volume_level / 2;
	}
	else
	{
		return volume_level / 4;
	}
}
	
uint8_t  ConvertCountToVolumeLevel(uint8_t count)
{
	if (detents_per_revolution == DETENTS_PER_REV::Fifty)
	{
		return count * 2;
	}
	else
	{
		return count * 4;
	}
}

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
		if (!_received_first_volume) // This is the initial volume setting
		{
			_volume_level = buffer[0];
			ESP_LOGI(HID_TAG, "Received first volume data: %d", _volume_level);
			_received_first_volume = true;
			return;
		}
		else // Volume has changed outside of this device or we're getting confirmation of a recent change origination here
		{
			uint8_t new_volume_level = buffer[0];
			//ESP_LOGI(HID_TAG, "Got feedback, _device_volume is %d and new volume received is %d", _volume_level, new_volume_level);
			
			if (new_volume_level == _volume_level) // This is just a message confirming our recent change
			{
				//ESP_LOGI(HID_TAG, "Received data is confirming recent change made by this device");
			}
			else // Volume has adjusted from outside
			{
				//ESP_LOGI(HID_TAG, "Received data is different value and change needs to be accounted for");
				SetVolumeLevelExternal(new_volume_level);
			}
		}
	}
	
	uint8_t GetVolumeLevel()
	{
		return _volume_level;
	}
	
	// Set the volume level from a change that occured outside the system
	void SetVolumeLevelExternal(uint8_t volume)
	{
		send_led_message_volume_level(volume); // Need to set LEDs
		uint8_t new_count = ConvertVolumeLevelToCount(volume);
		count = new_count; // Need to change count to reflect new status
		if (volume == 100) // If external changes have moved it to 100 or 0 percent then the angle limit needs to be set again
		{
			max_angle_threshold = sensor.getSensorAngle() * (180 / PI) - detent_width / 2;
			ESP_LOGI(MOTOR_TAG, "Volume changed from outside and is now at 100, setting CW threshold angle to %f", max_angle_threshold);
		}
		else if (volume == 0)
		{
			min_angle_threshold = sensor.getSensorAngle() * (180 / PI) + detent_width / 2;
			ESP_LOGI(MOTOR_TAG, "Volume changed from outside and is now at 0, setting CCW threshold angle to %f", min_angle_threshold);
		}
		_volume_level = volume;
	}
	
	// Set the volume level from a change that occured inside the system
	void SetVolumeLevelInternal(uint8_t volume) // Assumed to be 0...100 in increments of 2
	{
		// Need to send update over HID
		if (volume > _volume_level) // volume increment
		{
			uint8_t increment_counts = (volume - _volume_level) % 2;
			increment(increment_counts);
		}
		else // volume decrement
		{
			uint8_t decrement_counts = (_volume_level - volume) % 2;
			decrement(decrement_counts);
		}
		send_led_message_volume_level(volume); // Need to set LEDs
		_volume_level = volume;
	}
	
	void block_till_first_volume()
	{
		while (!_received_first_volume)
		{
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}

	uint16_t _onGetDescriptor(uint8_t* buffer) {
		memcpy(buffer, report_descriptor, sizeof(report_descriptor));
		return sizeof(report_descriptor);
	}

	bool increment(int counts = 1)
	{
		for (int k = 0; k < counts; k++)
		{
			uint8_t data[1];
			data[0] = 0x01;
			HID.SendReport(1, data, 1);
			data[0] = 0x00;
			return HID.SendReport(1, data, 1);	
			ESP_LOGI(VOLUME_TAG, "Incrementing volume");
		}
	}
	
	bool decrement(int counts = 1)
	{
		for (int k = 0; k < counts; k++)
		{
			uint8_t data[1];
			data[0] = 0x02;
			HID.SendReport(1, data, 1);
			data[0] = 0x00;
			return HID.SendReport(1, data, 1);
			ESP_LOGI(VOLUME_TAG, "Decrementing volume");
		}
	}

	bool send(uint8_t * value) {
		return HID.SendReport(0, value, 8);
	}
	
private:
	uint8_t _volume_level = 0;
	bool _received_first_volume = false;
};
CustomHIDDevice Device;

void count_change_internal_task(void* arg) // Receives new count value from queue
{
	int new_count;
	while (1)
	{
		if (xQueueReceive(count_queue_handle, &new_count, (TickType_t)5))
		{
			// how to handle negative count values? Should never occur in practice
			Device.SetVolumeLevelInternal(ConvertCountToVolumeLevel((uint8_t)new_count));
		}
	}
}

void button_handler_task(void* arg)
{
	uint32_t value;
	while (1)
	{
		if (xQueueReceive(button_message_queue, &value, (TickType_t)5))
		{
			//xSemaphoreTake(detent_ma_semaphore, portMAX_DELAY);
			int idx = value % (sizeof(detent_max_ma_values) / sizeof(detent_max_ma_values[0]));
			detent_max_ma = detent_max_ma_values[idx];
			ma_per_degree = detent_max_ma / detent_width;
			ESP_LOGI(BUTTON_TAG, "Got button press %d, setting current to idx %d, value %f", value, index, detent_max_ma);
			//xSemaphoreGive(detent_ma_semaphore);
		}
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

void setup_motor()
{
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

	ESP_LOGI(MOTOR_TAG, "Motor ready");
	vTaskDelay(pdMS_TO_TICKS(1000));
	init_angle = sensor.getSensorAngle() * (180 / PI);
	ESP_LOGI(MOTOR_TAG, "Init angle: %d, Max angle: %d", init_angle, max_angle);
}

//////////////////// LED Functions ///////////////////////////////////
void send_led_message_count(uint8_t count)
{
	xQueueSend(led_state_queue_handle, &count, (TickType_t)5);
}

void send_led_message_volume_level(uint8_t volume_level)
{
	send_led_message_count(ConvertVolumeLevelToCount(volume_level));
}
void led_control_task(void* arg) // Receives volume level in detent counts
{
	uint8_t count_received;
	while (1)
	{
		if(xQueueReceive(led_state_queue_handle, &count_received, (TickType_t)5))
		{
			float levels_per_led = (float)detents_per_revolution / (float)LED_COUNT;
			int fully_lit_count = (int)(count_received / levels_per_led);
			float remainder = fmod(count_received, levels_per_led);
			float last_led_percentage = remainder / levels_per_led;
			set_strip(fully_lit_count, last_led_percentage);
		}
		vTaskDelay(10);
	}
}

void set_strip(int fully_lit, float last_percent)
{
	strip.clear();
	for (int j = 0; j < fully_lit; j++)
	{
		strip.setPixelColor(led_order[j], 0, 255, 0);
	}
	if (last_percent > 0.001 && fully_lit < LED_COUNT) 
	{
		int brightness = (int)(last_percent * 255);
		int last_led_idx = led_order[fully_lit]; 
		strip.setPixelColor(last_led_idx, 0, brightness, 0);
	}
	strip.show();
}

void set_strip_single_color(uint8_t red, uint8_t green, uint8_t blue)
{
	for (int i = 0; i < LED_COUNT; i++)
	{
		strip.setPixelColor(i, red, green, blue);
	}
	strip.show();
}
//////////////////////////////////////////////////////////////////////

void setup()
{
	Serial.begin(115200);
	ESP_LOGI(MAIN_TAG, "Started setup");
	
	// Setup light sensor to set led strip
	setup_light_sensor();
	
	// Set strip up first to show init sequence
	strip.begin();
	strip.show();
	strip.clear();
	strip.setBrightness(get_best_led_level());
	set_strip_single_color(50, 0, 0);
	
	// Setup USB to begin waiting for first volume
	Device.begin();
	USB.begin();
	
	// This takes a while
	setup_motor();
	pinMode(4, OUTPUT);
	
	// Wait to get volume
	Device.block_till_first_volume();
	uint8_t initial_volume = Device.GetVolumeLevel();
	
	// Count starts at 0 and has range of 0...detents/revolution
	float volume_as_percent = (float)initial_volume / 100.0f;
	int initial_count = (int) (volume_as_percent * (float) detents_per_revolution);
	ESP_LOGI("MOTOR", "Setting inital count to %d", initial_count);
	count = initial_count;
	
	count_queue_handle = xQueueCreate(5, sizeof(int));
	led_state_queue_handle = xQueueCreate(5, sizeof(uint8_t));
	
	pinMode(button_pin, INPUT_PULLUP);
	detent_ma_semaphore =  xSemaphoreCreateMutex();
	button_message_queue = xQueueCreate(5, sizeof(uint32_t));
	attachInterrupt(button_pin, button_isr, FALLING);
	
	
	xTaskCreate(count_change_internal_task, "Count Change Internal Task", 2048, NULL, configMAX_PRIORITIES - 5, &count_change_internal_handle);
	xTaskCreate(led_control_task, "LED control task", 2048, NULL, configMAX_PRIORITIES - 6, &led_task_handle);
	xTaskCreate(light_monitor_task, "Light monitor task", 4096, NULL, configMAX_PRIORITIES - 10, &light_monitor_task_handle);
	xTaskCreate(button_handler_task, "Button monitor task", 2048, NULL, configMAX_PRIORITIES - 9, &button_task_handle);
	
	
	int temp_count = initial_count;
	xQueueSend(count_queue_handle, &temp_count, (TickType_t)5);
}

void loop()
{
	digitalWrite(4, HIGH);
	motor.loopFOC();
//	if(xSemaphoreTake(detent_ma_semaphore, (TickType_t)3) != pdTRUE)
//	{
//		return;
//	}
	
	float angle_rad = sensor.getAngle();
	float angle = angle_rad * (180 / PI);
	float spring_current = 0;
	float total_current = 0;
	
	float threshold = detent_width / 2;
	float angle_noise_threshold = 0.3;
	
	if (count == ConvertVolumeLevelToCount(100) && angle < max_angle_threshold)
	{
		float degrees_past_edge = abs(angle - max_angle_threshold);
		spring_current = (virtual_wall_ma_per_degree / 1000.0) * degrees_past_edge * -1;
		motor.move(spring_current);
		return;
	}
	else if (count == 0 && angle > min_angle_threshold)
	{
		float degrees_past_edge = abs(angle - min_angle_threshold);
		spring_current = (virtual_wall_ma_per_degree / 1000.0) * degrees_past_edge;
		motor.move(spring_current);
		return;
	}
	
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
	
	//xSemaphoreGive()
	
	bool at_snap_edge = abs(last_current) > peak_current_threshold && abs(total_current) > peak_current_threshold; // check that the transition that may be occuring is at the snap edge and not in the settling point between snaps
	if (last_current > 0 && total_current < 0 && at_snap_edge) // CCW snap
	{
		count--;
		if (count == 0)
		{	
			min_angle_threshold = angle + (detent_width/2);
			ESP_LOGI(MOTOR_TAG, "Reached min count: %d at angle: %f, setting CCW threshold to %f", count, angle, min_angle_threshold);
		}
		
		ESP_LOGI(MOTOR_TAG, "Detected decrement, count value now: %d, angle: %f",count, angle);
		xQueueSend(count_queue_handle, &count, (TickType_t)5);
	}
	else if(last_current < 0 && total_current > 0 && at_snap_edge) // CW snap
	{
		count++;
		if (count == ConvertVolumeLevelToCount(100))
		{
			max_angle_threshold = angle - (detent_width/2); // at snap angle, it settles detent_width/2 further
			ESP_LOGI(MOTOR_TAG, "Reached max count: %d at angle: %f, setting CW threshold to %f", count, angle, max_angle_threshold);
		}
		ESP_LOGI(MOTOR_TAG, "Detected increment, count value now: %d, angle: %f", count, angle);
		xQueueSend(count_queue_handle, &count, (TickType_t)5);
	}
	
	last_current = total_current;
	motor.move(total_current);
	digitalWrite(4, LOW);
}