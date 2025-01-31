#include <Arduino.h>
#include <RF24.h>
#include <U8g2lib.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <CRC32.h>
#include <Wire.h>

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);   // Adafruit Feather M0 Basic Proto + FeatherWing OLED
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);   // Adafruit Feather M0 Basic Proto + FeatherWing OLED

#define DIP0 A3
#define DIP1 10
#define DIP2 9
#define DIP3 11

#define PWM_OUTPUT 3

#define BTN_UP A1
#define BTN_DOWN A0
#define BTN_SELECT A2
#define MENU_LEFT_ICON 0, 64
#define MENU_MIDDLE_ICON 62, 64
#define MENU_RIGHT_ICON 120, 64

Bounce2::Button btnUp = Bounce2::Button();
Bounce2::Button btnDown = Bounce2::Button();
Bounce2::Button btnSelect = Bounce2::Button();


RF24 radio(10, 9); // CE, CSN

byte address[6] = "52350";

#define SCREEN_MAIN 0
#define SCREEN_MENU 1
#define SCREEN_PARAMETER 2
#define MODE_AUTO 0
#define MODE_OFF 1
#define MODE_ON 2

struct {
  bool redraw : 1;
  uint8_t screen: 2;
} operation;

uint8_t powerOutput;
uint8_t powerRadio;
uint16_t luxValue;
uint8_t luxSimple;

struct __attribute__((packed)) StoreData {
  uint8_t addressProtocol;
  uint8_t fadeSpeed;
  uint8_t mode;
  uint8_t powerSet;
  uint8_t lowLightThreshould;
};

StoreData data;

// Function to calculate CRC32 for the struct
uint32_t calculateCRC(const StoreData& data) {
  CRC32 crc;
  crc.update((const uint8_t*)&data, sizeof(data)); // Calculate CRC from the bytes of the struct
  return crc.finalize(); // Return the final CRC32 value
}


void configSave() {
  int32_t crc = calculateCRC(data);
  EEPROM.put(0, data);           // Write struct at address 0
  EEPROM.put(sizeof(data), crc); // Write CRC checksum after the struct
}

void configLoad() {
  EEPROM.begin();
  EEPROM.get(0, data);

  uint32_t crc = calculateCRC(data);
  uint32_t savedCRC;
  EEPROM.get(sizeof(data), savedCRC);
  
  if (savedCRC != crc || data.mode > 2 ) {
    Serial.print("inilizing EEPROM.");
    data.mode = MODE_AUTO;
    data.addressProtocol = 0;
    data.fadeSpeed = 1;
    data.powerSet = 0;
    configSave();
  } else {
    Serial.println("EEPROM is ok.");
  }
}

struct __attribute__((packed)) RemoteLight {
    unsigned long time;
    uint8_t address;
    uint8_t power;
};

struct MenuItem {
  const char* label;
  uint8_t* value;
  const uint8_t min_value;
  const uint8_t max_value;
  MenuItem(const char* label, uint8_t* value, const uint8_t min_value, const uint8_t max_value) : 
    label(label), 
    value(value), 
    min_value(min_value), 
    max_value(max_value) {}
};

uint8_t currentItem;

const MenuItem menu[]  = {
  MenuItem("Address", &data.addressProtocol, 1, 254),
  MenuItem("Fade speed", &data.fadeSpeed, 1, 10),
  MenuItem("Low light", &data.lowLightThreshould, 0, 255),
  MenuItem("Exit", NULL, 0, 0)
};
#define MENU_LENGTH 3

char printBuff[20] = {0};

void initRfReading() {
  address[sizeof(address)-1] = '0' + data.addressProtocol;
  radio.setChannel(23);
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_8);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.startListening();

}

void setup()
{
  Serial.begin(9600);
  Serial.println("Startup");

  configLoad();
  powerOutput = 0;
  powerRadio = 0;
  operation.redraw = 0;
  currentItem = 0;
  operation.screen = SCREEN_MAIN;

  btnUp.attach(BTN_UP, INPUT_PULLUP); 
  btnUp.interval(5); // debounce interval in milliseconds

  btnDown.attach(BTN_DOWN, INPUT_PULLUP); 
  btnDown.interval(5); // debounce interval in milliseconds

  btnSelect.attach(BTN_SELECT, INPUT_PULLUP); 
  btnSelect.interval(5); // debounce interval in milliseconds

  Serial.println("P0");
  u8g2.begin();  // Initialize the display
  Serial.println("P1");

  u8g2.clearBuffer();  // Clear the internal buffer
  Serial.println("P2");

  u8g2.setFont(u8g2_font_7x14_tf);  // Select a font

  u8g2.sendBuffer();  // Send the buffer to the display  

  
  pinMode(3, OUTPUT);
  //pinMode(BTN_UP, INPUT_PULLUP);
  //pinMode(BTN_DOWN, INPUT_PULLUP);
  //pinMode(BTN_SELECT, INPUT_PULLUP);


  // disable pull up
  PORTB &= ~(1 << PB1);
  Serial.println("P3");
  
  /*
  DDRB |= _BV(PB1);
  TCCR1A |= _BV(COM1A1) | _BV(WGM10);
	TCCR1B |= _BV(CS10) | _BV(WGM12);
  */
  u8g2.clearBuffer();
  Serial.println("P4");

  u8g2.setFont(u8g2_font_ncenB08_tr);
  
  Serial.println("Setup radio");

  radio.begin();
  if (!radio.isChipConnected()) {
    Serial.println("Setup radio is failed");

    u8g2.drawStr(0, 16, "RF24 fail");  // Draw text
    while (1){};
  } else {
    Serial.println("Radio is connected");
    initRfReading();
  }
  Serial.println("Radio setup is done");
  analogWrite(PWM_OUTPUT, 0); 

  operation.redraw = true;
  Serial.println("Setup is done");

  Wire.beginTransmission(0x44);
  Wire.write(0x01);
  //Wire.write(0b11001100);
  //Wire.write(0b00000000);
  Wire.write(0b10011100);  // 5.12 lux/bit + 
  Wire.write(0b00000100);  // ME=1
  Wire.endTransmission(true);

}


void check_events(void) {
  btnUp.update();
  btnDown.update();
  btnSelect.update();
}


void ui_main() {
  if (btnUp.pressed()) {
    if (data.mode < MODE_ON) {
      data.mode ++;
      configSave();
      operation.redraw = true;
      Serial.println("Main UP");
    }
  }

  if (btnDown.pressed()) {
    if (data.mode > 0) {
      data.mode --;
      configSave();
      operation.redraw = true;
      Serial.println("Main Down");
    }
  }

  if (btnSelect.pressed()) {
    operation.screen = SCREEN_MENU;
    currentItem = 0;
    operation.redraw = true;
    Serial.println("Main enter");
    return;
  }

  if (!operation.redraw) {
    return;
  }

  Serial.println("Main redraw");


  u8g2.clearBuffer();
  u8g2.firstPage();
  do {
    // Draw Radio
    u8g2.setFont(u8g2_font_open_iconic_www_2x_t);
    u8g2.drawStr(0, 16, "\x48");  // Draw text

    // Draw mode
    switch (data.mode) {
      case MODE_AUTO:
        u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);
        u8g2.drawStr(64, 16, "\x57");
        break;
      case MODE_ON:
        u8g2.setFont(u8g2_font_open_iconic_check_2x_t);
        u8g2.drawStr(64, 16, "\x41");
        data.powerSet = 255;
        break;
      case MODE_OFF:
        u8g2.setFont(u8g2_font_open_iconic_check_2x_t);
        u8g2.drawStr(64, 16, "\x42");
        data.powerSet = 0;
        break;
    }

    memset(printBuff, '\0', sizeof(printBuff));
    snprintf(printBuff, sizeof(printBuff), "C:%i T:%i ", powerOutput, data.powerSet);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 32);
    u8g2.print(printBuff); 

    memset(printBuff, '\0', sizeof(printBuff));
    snprintf(printBuff, sizeof(printBuff), "Lux:%u", luxValue);
    u8g2.setCursor(0, 42);
    u8g2.print(printBuff); 

    if (luxSimple > data.lowLightThreshould) {
      u8g2.drawStr(100, 42, "S");
    } else {
      u8g2.drawStr(100, 42, "F");
    }

  } while ( u8g2.nextPage() );
  Serial.println("Main redraw 3");
  operation.redraw = false;
}


void ui_menu() {
  if (btnUp.pressed()) {
    if (currentItem < MENU_LENGTH) {
      currentItem ++;
      operation.redraw = true;
    }
  }

  if (btnDown.pressed()) {
    if (currentItem > 0) {
      currentItem --;
      operation.redraw = true;
    }
  }

  if (btnSelect.pressed()) {
    if (!menu[currentItem].value) {
      operation.screen = SCREEN_MAIN;
      operation.redraw = true;
      return;
    } else {
      operation.screen = SCREEN_PARAMETER;
      operation.redraw = true;
      return;
    }
  }

  if (!operation.redraw) {
    return;
  }
  Serial.println("Menu redraw");

  const MenuItem current = menu[currentItem];

  u8g2.firstPage();
  u8g2.clearBuffer();

  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(32, 25);
    u8g2.print("M E N U");

    u8g2.setCursor(16, 40);
    u8g2.print(current.label);
 
    u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
    if (currentItem > 0) {
        u8g2.drawStr(MENU_LEFT_ICON, "\x51");
    }
    
    if (currentItem < MENU_LENGTH) {
        u8g2.drawStr(MENU_MIDDLE_ICON, "\x52");
    }

    if (!menu[currentItem].value) { 
      u8g2.drawStr(MENU_RIGHT_ICON, "\x5A");
    } else { 
      u8g2.drawStr(MENU_RIGHT_ICON, "\x53");
    }

  } while ( u8g2.nextPage() );
  operation.redraw = false;
}


void ui_parameter() {
  if (btnUp.pressed()) {
    if (*menu[currentItem].value < menu[currentItem].max_value) {
      (*menu[currentItem].value)++;
      operation.redraw = true;
      Serial.println("Parameter Up");
    }
  }
  if (btnDown.pressed()) {
    if (*menu[currentItem].value > menu[currentItem].min_value) {
      (*menu[currentItem].value)--;
      operation.redraw = true;
      Serial.println("Parameter Down");
    }
  }

  if (btnSelect.pressed()) {
    configSave();
    initRfReading();
    operation.screen = SCREEN_MENU;
    operation.redraw = true;
    Serial.println("Parameter Select");
    return;
  }

  if (!operation.redraw) {
    return;
  }
  
  Serial.println("Parameter redraw");
  const MenuItem current = menu[currentItem];

  memset(printBuff, '\0', sizeof(printBuff));
  snprintf(printBuff, sizeof(printBuff), "%i", *menu[currentItem].value);

  u8g2.firstPage();
  u8g2.clearBuffer();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(16, 32);
    u8g2.print(current.label);

    u8g2.setCursor(16, 48);
    u8g2.print(printBuff); 


    if (*menu[currentItem].value < menu[currentItem].max_value) {
      u8g2.drawStr(MENU_MIDDLE_ICON, "+");
    }
    if (*menu[currentItem].value > menu[currentItem].min_value) {
      u8g2.drawStr(MENU_LEFT_ICON, "-");
    }
    u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
    u8g2.drawStr(MENU_RIGHT_ICON, "\x5A");

  } while ( u8g2.nextPage() );
  operation.redraw = false;
}
 

uint32_t convertToLux(uint16_t resultRegister) {
    // Extract exponent (E[3:0]) and mantissa (R[11:0]) from the result register
    uint8_t exponent = (resultRegister >> 12) & 0x0F;  // Bits 15:12
    uint16_t mantissa = resultRegister & 0x0FFF;       // Bits 11:0

    // Calculate lux: lux = 0.01 × (2^exponent) × mantissa
    uint32_t lux = (uint32_t)(0.01 * (1 << exponent) * mantissa); 

    return lux;
}

#define OPT3001_ADDRESS 0x44 
#define RESULT_REGISTER 0x00

// Function to read the result register from OPT3001
uint16_t readResultRegister() {
    uint16_t result = 0;

    // Start communication
    Wire.beginTransmission(OPT3001_ADDRESS);
    Wire.write(RESULT_REGISTER); // Request the result register
    Wire.endTransmission(false); // Restart communication without releasing the bus

    // Request 2 bytes of data from the sensor
    Wire.requestFrom(OPT3001_ADDRESS, 2);

    if (Wire.available() == 2) { // Ensure we received 2 bytes
        uint8_t msb = Wire.read(); // Read the most significant byte
        uint8_t lsb = Wire.read(); // Read the least significant byte
        result = (msb << 8) | lsb; // Combine the two bytes into a 16-bit value
    }

    return result;
}


void loop(void) {
  check_events(); // check for button press with bounce2 library

  if (data.mode == MODE_AUTO) {
    data.powerSet = powerRadio;
  }

  if (powerOutput < data.powerSet) {
    powerOutput ++;
    if (operation.screen == SCREEN_MAIN) {
      operation.redraw = true;
    }

  } else if (powerOutput > data.powerSet) {
    powerOutput --;
    if (operation.screen == SCREEN_MAIN) {
      operation.redraw = true;
    }
  }


  switch (operation.screen) {
    case SCREEN_MAIN:
      ui_main();
      break;
    case SCREEN_MENU:
      ui_menu();
      break;
    case SCREEN_PARAMETER:
      ui_parameter();
      break;
  }

  if (radio.available()) {
    Serial.println("Data is available");

    RemoteLight data;
    radio.read(&data, sizeof(data));
    
    if (data.power == 100) {
      powerRadio = 255; 
    } else if (data.power == 0) {
      powerRadio = 0; 
    } else {
      powerRadio = 255.0 / 100.0 * (float)data.power;
    }
  }
  Serial.println("-------------"); 
  Serial.println("Sensor"); 

  luxValue = readResultRegister();
  operation.redraw = true;
  Serial.print("Lux value: ");
  Serial.println(luxValue);
  Serial.println("-------------"); 

  luxSimple = (luxValue & 0xFF00) >> 8;
  Serial.print("Lux simple: ");
  Serial.println(luxSimple);
  Serial.println("-------------"); 
  if (luxSimple > data.lowLightThreshould) {
    analogWrite(PWM_OUTPUT, 0);
  } else {
    analogWrite(PWM_OUTPUT, powerOutput);
  }
}
