#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <WiFi.h>
#include <vector>
#include <esp_task_wdt.h>
#include <WebServer_ESP32_ENC.h>

using namespace std;
// WiFi settings
const char* ssid = "RSAWEB-IoT";
const char* password = "vs0w6g5GHkwT5Afb";
IPAddress serverIP(192, 168, 1, 105);



#if !( defined(ESP32) )
  #error This code is designed for (ESP32 + ENC28J60) to run on ESP32 platform! Please check your Tools->Board setting.
#endif

// LAN Module Connections and Settings
#define DEBUG_ETHERNET_WEBSERVER_PORT       Serial

// Debug Level from 0 to 4
#define _ETHERNET_WEBSERVER_LOGLEVEL_       3

////////////////////////////////////////////////////////
//00 18 00 00 00 23 E3 03 20 00 00 00 00 00 03 00 00 00 00 13 83 00 C8 FF FF 00 00 00 00 14 DC 00 05 00 01 00 00 03 E8 00
//1   2  3  4
//Optional values to override default settings
#define ETH_SPI_HOST        SPI2_HOST
#define SPI_CLOCK_MHZ       20

//Must connect INT to GPIOxx or not working
#define INT_GPIO            4

#define MISO_GPIO           13
#define MOSI_GPIO           12
#define SCK_GPIO            17
#define CS_GPIO             15


// Modbus settings
const uint16_t PORT = 502;
const uint16_t MAX_READ_REGS = 125;
const uint16_t MAX_WRITE_REGS = 123;

// Modbus addresses
const uint16_t REG_INPUT_START = 1000;
const uint16_t REG_INPUT_NREGS = 4;
const uint16_t REG_HOLDING_START = 2000;
const uint16_t REG_HOLDING_NREGS = 4;

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6

// Enter a MAC address and IP address for your controller below.
#define NUMBER_OF_MAC      20

byte mac[][NUMBER_OF_MAC] =
{
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x02 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x03 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x04 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x05 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x06 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x07 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x08 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x09 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0A },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0B },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0C },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0D },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0E },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0F },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x10 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x11 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x12 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x13 },
  { 0x78, 0x21, 0x84, 0x88, 0x37, 0x13 },
};

// Select the IP address according to your local network
// Select the IP address according to your local network
IPAddress myIP(192, 168, 1, 110);
IPAddress myGW(192, 168, 1, 1);
IPAddress mySN(255, 255, 255, 0);

// Google DNS Server IP
IPAddress myDNS(8, 8, 8, 8);



//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

void do_send(osjob_t* j);
void Task1code( void * pvParameters );
void Task2code( void * pvParameters );
TaskHandle_t Task1;
TaskHandle_t Task2;
uint8_t readHoldingRegsResponse[41];
uint8_t data_to_send[35];
uint8_t frame[12] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0xE3, 0x03, 0x00, 0x10, 0x00, 0x10};
WiFiClient client;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= {  0x31, 0x7A, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70   };

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={  0x32, 0x7A, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70   };

void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x75, 0x7F, 0xE4, 0xB5, 0xD0, 0xBB, 0x39,
                                         0x03, 0x9D, 0x24, 0x55, 0x35, 0xD8, 0x42,
                                         0xCE, 0xD7 };


void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "RSAWEB";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};


void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
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

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, readHoldingRegsResponse, sizeof(readHoldingRegsResponse)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));
    //esp_task_wdt_delete();

     // To be called before ETH.begin()
    ESP32_ENC_onEvent();
    ETH.begin( MISO_GPIO, MOSI_GPIO, SCK_GPIO, CS_GPIO, INT_GPIO, SPI_CLOCK_MHZ, ETH_SPI_HOST );
    ETH.config(myIP, myGW, mySN, myDNS);
    ESP32_ENC_waitForConnect();

    // WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED) {
    // delay(1000);
    // Serial.println("Connecting to WiFi...");
    // }

    // Serial.println("WiFi connected");
    // Serial.println("IP address: ");
    // Serial.println(WiFi.localIP());

    // Connect to Modbus server
    
    while (!client.connect(serverIP, PORT)) {
    Serial.println("Modbus connection failed");
    delay(1000);
    }

    Serial.println("Modbus connected");
    
    delay(20);

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);
    LMIC_setLinkCheckMode(0);   // Disable link check validation
    //LMIC_setAdrMode(false);     // Disable ADR
    LMIC.dn2Dr = DR_SF9;        // TTN uses SF9 for its RX2 window.
    LMIC_setDrTxpow(DR_SF7, 14);
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
    xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 5, &Task1,  1); 

    delay(500); 

    //xTaskCreatePinnedToCore(Task2code, "Task1", 10000, NULL, 5, &Task2,  1); 

    delay(500);
}

void Task1code( void * pvParameters ){
  esp_task_wdt_init(30, false);
  Serial.print("Task1 running on core ");

  Serial.println(xPortGetCoreID());
  esp_task_wdt_delete(Task1);

  for(;;){

    os_runloop_once();

  } 

}

void Task2code( void * pvParameters ){
  esp_task_wdt_init(30, false);
  Serial.print("Task2 running on core ");

  Serial.println(xPortGetCoreID());
  
  
  

  for(;;){

    client.write(frame, 12);
    if(client.available())
    {
        client.readBytes(readHoldingRegsResponse, sizeof(readHoldingRegsResponse));
        //Serial.print("Reg ");
        int j = 0;
        for (int i = 6; i < sizeof(readHoldingRegsResponse); i++) {
            // uint16_t val = (readHoldingRegsResponse[9 + i*2] << 8) | readHoldingRegsResponse[10 + i*2];
            data_to_send[j] = readHoldingRegsResponse[i];
            //Serial.printf("%02X", readHoldingRegsResponse[i]);
            //Serial.print(" ");
            j++;
        }

        //memset(readHoldingRegsResponse, 0, sizeof(readHoldingRegsResponse));
        //
        Serial.println();
        //client.flush();
        delay(2000);
    }

  } 

}


void loop() {

esp_task_wdt_init(30, false);    
//   //Serial.print("Task1 running on core ");
//   //Serial.println(xPortGetCoreID());
//   os_runloop_once();


while(1){
    frame[1] += 1;
    client.write(frame, 12);
    delay(20);
    client.readBytes(readHoldingRegsResponse, sizeof(readHoldingRegsResponse));
    //Serial.print("Reg ");
        int j = 0;
        for (int i = 9; i < sizeof(readHoldingRegsResponse); i++) {
            // uint16_t val = (readHoldingRegsResponse[9 + i*2] << 8) | readHoldingRegsResponse[10 + i*2];
            data_to_send[j] = readHoldingRegsResponse[i];
            //Serial.printf("%02X", readHoldingRegsResponse[i]);
            //Serial.print(" ");
            j++;
        }

        //memset(readHoldingRegsResponse, 0, sizeof(readHoldingRegsResponse));
        //
        //Serial.println();
        //client.flush();
        delay(2000);

}

    
    // if(client.available())
    // {
    //     client.readBytes(readHoldingRegsResponse, sizeof(readHoldingRegsResponse));
    //     Serial.print("Reg ");
    //     int j = 0;
    //     for (int i = 6; i < sizeof(readHoldingRegsResponse); i++) {
    //         // uint16_t val = (readHoldingRegsResponse[9 + i*2] << 8) | readHoldingRegsResponse[10 + i*2];
    //         data_to_send[j] = readHoldingRegsResponse[i];
    //         Serial.printf("%02X", readHoldingRegsResponse[i]);
    //         Serial.print(" ");
    //         j++;
    //     }

    //     //memset(readHoldingRegsResponse, 0, sizeof(readHoldingRegsResponse));
    //     //
    //     Serial.println();
    //     //client.flush();
    //     delay(2000);
    // }
}