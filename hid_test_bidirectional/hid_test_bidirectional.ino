#if ARDUINO_USB_MODE
#warning This sketch should be used when USB is in OTG mode
void setup(){}
void loop(){}
#else
#include "USB.h"
#include "USBHID.h"
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

class CustomHIDDevice: public USBHIDDevice {
public:
  CustomHIDDevice(void){
    static bool initialized = false;
    if(!initialized){
      initialized = true;
      HID.addDevice(this, sizeof(report_descriptor));
    }
  }
  
  void begin(void){
    HID.begin();
  }

  void _onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len) override
  {
    Serial.println("Received report");
    Serial.println("Report ID: " + String(report_id));
    Serial.println("Report Length:" + String(len));
    for(int i = 0; i < len; i++){
      Serial.print(buffer[i], HEX);
    }
    Serial.println();
  }
    
  uint16_t _onGetDescriptor(uint8_t* buffer){
    memcpy(buffer, report_descriptor, sizeof(report_descriptor));
    return sizeof(report_descriptor);
  }

  bool increment()
  {
    uint8_t data[1];
    data[0] = 0x01;
    return HID.SendReport(1, data, 1);
  }

  bool send(uint8_t * value){
    return HID.SendReport(0, value, 8);
  }
};

CustomHIDDevice Device;
#endif /* ARDUINO_USB_MODE */

const int buttonPin = 0;
int previousButtonState = HIGH;
uint8_t axis[8];

// void event_handler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
// {
//   Serial.println("Received event");
// }

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  pinMode(buttonPin, INPUT_PULLUP);
  //HID.onEvent(event_handler);
  Device.begin();
  USB.begin();
}

void loop() {
  int buttonState = digitalRead(buttonPin);
  if (HID.ready() && buttonState != previousButtonState) {
    previousButtonState = buttonState;
    if (buttonState == LOW) {
      Serial.println("Button Pressed");
      Device.increment();
    } else {
      Serial.println("Button Released");
    }
    delay(100);
  }
}
