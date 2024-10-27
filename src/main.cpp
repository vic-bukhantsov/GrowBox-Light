#include <Arduino.h>
#include <RF24.h>
#include <U8g2lib.h>
#include <Bounce2.h>

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);   // Adafruit Feather M0 Basic Proto + FeatherWing OLED
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0);   // Adafruit Feather M0 Basic Proto + FeatherWing OLED

#define DIP0 A3
#define DIP1 10
#define DIP2 9
#define DIP3 11

#define PWM_OUTPUT 3

#define BTN_UP A0
#define BTN_DOWN A1
#define BTN_SELECT A2

Bounce2::Button btnUp = Bounce2::Button();
Bounce2::Button btnDown = Bounce2::Button();
Bounce2::Button btnSelect = Bounce2::Button();


RF24 radio(10, 9); // CE, CSN

const byte address[6] = "52351";

#define SCREEN_MAIN 0
#define SCREEN_MENU 1
#define SCREEN_PARAMETER 2
#define MODE_AUTO 0
#define MODE_OFF 1
#define MODE_ON 2

struct {
  bool redraw : 1;
  uint8_t screen: 2;
  uint8_t mode: 2;
} operation;


uint8_t powerOutput;
uint8_t powerSet;


struct __attribute__((packed)) RemoteLight {
    unsigned long time;
    uint8_t address;
    uint8_t power;
};


struct MenuItem {
  const char* label;
  const uint8_t* value;
  const uint8_t min_value;
  const uint8_t max_value;
  MenuItem(const char* label, const uint8_t* value, const uint8_t min_value, const uint8_t max_value) : 
    label(label), 
    value(value), 
    min_value(min_value), 
    max_value(max_value) {}
};

uint8_t protocolAddress;
uint8_t speedFade;

uint8_t currentItem;

const MenuItem menu[] = {
  MenuItem("Address", &protocolAddress, 1, 254),
  MenuItem("Fade speed", &speedFade, 1, 10),
  MenuItem("Exit", NULL, 0, 0)
};


bool pinOnTimer = false;

void setup()
{
  powerOutput = 0;
  powerSet = 0;
  operation.redraw = 0;
  currentItem = 0;
  operation.screen = SCREEN_MAIN;
  operation.mode = MODE_AUTO;

  Serial.begin(9600);
  Serial.println("Hello, Serial Console!");

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

    radio.setChannel(23);
    radio.openReadingPipe(1, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_250KBPS);
    radio.setCRCLength(RF24_CRC_8);
    radio.enableAckPayload();
    radio.enableDynamicPayloads();
    radio.startListening();
    u8g2.setFont(u8g2_font_open_iconic_www_2x_t);
    u8g2.drawStr(0, 16, "\x48");  // Draw text
  }
  Serial.println("Radio setup is done");

  u8g2.sendBuffer();
  analogWrite(PWM_OUTPUT, 0);

  Serial.println("Setup is done");
}


void check_events(void) {
  btnUp.update();
  btnDown.update();
  btnSelect.update();
}

/*
void print_power(uint8_t power) {
  char powerLine[32];
  snprintf(powerLine, sizeof(powerLine), "Power: %i%%", 100 * power / 255 );
  u8g2.setCursor(0, 20);
  u8g2.print(powerLine);
  u8g2.sendBuffer();
}
*/;

int loop_idx = 0;




void handle_events(void) {
  // 0 = not pushed, 1 = pushed  
  /*
  if (selectBtn.pressed()) {
    if (!mui.isFormActive()) {
      mui.gotoForm(form_id=1, initial_cursor_position=0);
    } else {
      mui.sendSelect();
    }
    is_redraw = 1;
  } else if (nextBtn.pressed()) {
    if (!mui.isFormActive()) {
      loop_idx ++;
    } else {

    }
    is_redraw = 1;
  }
  else if (prevBtn.pressed()) {
    loop_idx --;
    is_redraw = 1;
  }    
  */
}


void ui_main() {
  if (btnUp.pressed()) {
    if (operation.mode < MODE_ON) {
      operation.mode ++;
    }
  }

  if (btnDown.pressed()) {
    if (operation.mode > 0) {
      operation.mode --;
    }
  }

  if (btnSelect.pressed()) {
    operation.screen = SCREEN_MENU;
    currentItem = 0;
    operation.redraw = true;
    return;
  }

  u8g2.firstPage();

  do {
    // Draw Radio
    u8g2.setFont(u8g2_font_open_iconic_www_2x_t);
    u8g2.drawStr(0, 16, "\x48");  // Draw text

    // Draw mode
    switch (operation.mode) {
      case MODE_AUTO:
        u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);
        u8g2.drawStr(64, 16, "\x57");
        break;
      case MODE_ON:
        u8g2.setFont(u8g2_font_open_iconic_check_2x_t);
        u8g2.drawStr(64, 16, "\x41");
        powerSet = 255;
        break;
      case MODE_OFF:
        u8g2.setFont(u8g2_font_open_iconic_check_2x_t);
        u8g2.drawStr(64, 16, "\x42");
        powerSet = 0;
        break;
    }

    char powerLine[64] = {0};

    snprintf(powerLine, sizeof(powerLine), "C:%i T:%i ", powerOutput, powerSet);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 32);
    u8g2.print(powerLine); 
  } while ( u8g2.nextPage() );
}

void ui_menu() {

  if (btnUp.pressed()) {
    if (currentItem < sizeof(menu)-2) {
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
      return;
    }
  }

  if (!operation.redraw) {
    return;
  }

  const MenuItem current = menu[currentItem];

  u8g2.firstPage();
  u8g2.clearBuffer();

  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 64);
    u8g2.print("Menu");

    u8g2.setCursor(16, 32);
    u8g2.print(current.label);

  } while ( u8g2.nextPage() );
  operation.redraw = false;
}

void ui_parameter() {

}
 

void loop(void) {
  check_events(); // check for button press with bounce2 library

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

  //handle_events();  // process events from bounce2 library

  char powerLine[64] = {0};
  int has_data = 0;

  if (radio.available()) {
    Serial.println("Data is available");

    RemoteLight data;
    radio.read(&data, sizeof(data));
    has_data = 1;
    
    Serial.println(powerLine);

    /*display.setCursor(0, 8);
    display.fillRect(0, 8, SCREEN_WIDTH, 8, SSD1306_BLACK);
    display.print(line);
    display.display();  
    */
 
    if (data.power == 100) {
      /* 
      digitalWrite(9, HIGH);
      pinOnTimer = false;
      print_power(255); 
      */
      analogWrite(3, 255);
    } else if (data.power == 0) {
      /* 
      digitalWrite(9, LOW);
      pinOnTimer = false;
      print_power(0);
      */
      analogWrite(3, 0);
    } else {
      /*
      if (!pinOnTimer) {
        TCCR1A |= _BV(COM1A1) | _BV(WGM10);
        pinOnTimer = true;
      }
      */
      uint8_t power = 255.0 / 100.0 * (float)data.power;
      // OCR1A = power;
      Serial.println(power);
      //print_power(power);
      analogWrite(3, power);
    }
  }

  if (powerOutput < powerSet) {
    powerOutput ++;
  } else if (powerOutput > powerSet) {
    powerOutput --;
  }

  analogWrite(PWM_OUTPUT, powerOutput);


}