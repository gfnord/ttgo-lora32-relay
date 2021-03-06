#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>
#include "soc/efuse_reg.h"

#define LEDPIN 25

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 21
#define OLED_SCL 22

#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  30        //Time ESP32 will go to sleep (in seconds)

int16_t counter = 0;
int relay0 = 0;

SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

// Lora keys
static u1_t NWKSKEY[16] = { 0x48, 0x58, 0x3C, 0xCF, 0x76, 0xCA, 0xD6, 0xC7, 0xB9, 0x79, 0x36, 0x5D, 0xA3, 0x57, 0x15, 0x54 };  // Paste here the key in MSB format
static u1_t APPSKEY[16] = { 0xCA, 0x84, 0x63, 0x17, 0x8D, 0x2D, 0xD3, 0xD2, 0x62, 0xAE, 0x06, 0xFF, 0xFB, 0x69, 0x3C, 0x50 };  // Paste here the key in MSB format

static u4_t DEVADDR = 0x0286f74a;   

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;
char TTN_response[64];

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void lora_init() {
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Set static session parameters.
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Define sub-band
  LMIC_selectSubBand(1);

  // Set data rate (SF) and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7, 14);
}

void do_send(osjob_t* j){

  // Get the status of the pin
  relay0 = digitalRead(LEDPIN);

  // Payload
  uint8_t buffer[2];
  buffer[0] = relay0;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
      Serial.print(F("Sending uplink packet..."));
      Serial.println(counter);
      Serial.print(F("Relay Status:"));
      if(relay0){
        Serial.println(F("ON"));
      } else {
        Serial.println(F("OFF"));
      }
      display.clear();
      display.drawString (0, 0, "Sending uplink packet...");
      display.drawString (0, 20, "Relay Status:");
      if(relay0){
        display.drawString (70, 20, "ON");
      } else {
        display.drawString (70, 20, "OFF");
      }
      display.drawString (0, 50, String (counter));
      display.display ();
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
    case EV_TXCOMPLETE:
      display.clear();
      display.drawString (0, 0, "EV_TXCOMPLETE event!");
      relay0 = digitalRead(LEDPIN);
      display.drawString (0, 20, "Relay Status:");
      if(relay0){
        display.drawString (70, 20, "ON");
      } else {
        display.drawString (70, 20, "OFF");
      }
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received ack"));
        display.drawString (0, 20, "Received ACK.");
      }
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print(F("Data Received (Port and Data): "));
        int i;
        int downlink_port;
        downlink_port = LMIC.frame[LMIC.dataBeg-1];
        Serial.print(downlink_port);
        Serial.print(F(" "));
        char downlink_payload[LMIC.dataLen];
        for ( i = 0 ; i < LMIC.dataLen ; i++ )
          downlink_payload[i] = LMIC.frame[LMIC.dataBeg+i];
        downlink_payload[i] = 0;
        Serial.println(downlink_payload);
        // What to do with the downlink payload
        if (downlink_port == 9){
          if (*downlink_payload == '1') {
            // Turn on relay
            digitalWrite(LEDPIN, HIGH);
          }
          if (*downlink_payload == '0') {
            // Turn on relay
            digitalWrite(LEDPIN, LOW);
          }
        }  
        display.drawString (0, 30, "Received DATA.");
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      display.drawString (0, 50, String (counter));
      display.display ();
      counter++;
      Serial.println("Waiting the next scheduled package.");
      break;

    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
       break;

  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(1500);   // Give time for the seral monitor to start up
  Serial.println(F("Starting..."));

  // Use the Blue pin to signal transmission.
  pinMode(LEDPIN,OUTPUT);

  // Set the default state 
  digitalWrite(LEDPIN, LOW);

  // reset the OLED
  pinMode(OLED_RESET,OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(50);
  digitalWrite(OLED_RESET, HIGH);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.drawString(0, 0, "Init!");
  display.display();

  lora_init();

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}