#include <Arduino.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <WiFi.h>
#include <vector>
#include <esp_task_wdt.h>
//#include <SoftwareSerial.h>

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6

#define MODBUS_DIR_PIN  15 // connect DR, RE pin of MAX485 to gpio 4
#define MODBUS_RX_PIN 16 // Rx pin  
#define MODBUS_TX_PIN 17 // Tx pin 
#define MODBUS_SERIAL_BAUD 9600 // Baud rate for esp32 and max485 communication




#define BATT_CAPACITY                   0x00B8
#define BATT_VOLTAGE                    0x00B7
#define BATT_TEMP                       0x00B6
#define BATT_OUTPUT_POWER               0x00BE
#define BATT_OUTPUT_CURRENT             0x00BF
#define LOAD_VOLTAGE_L1                 0x009D
#define LOAD_VOLTAGE_L2                 0x009E
#define GRID_CURRENT_L1                 0x00A0
#define GRID_CURRENT_L2                 0x00A1
#define INVERTER_OUTPUT_CURRENT_L1      0x00A4
#define INVERTER_OUTPUT_CURRENT_L2      0x00A5

uint16_t data_register[] = { BATT_CAPACITY, BATT_VOLTAGE, BATT_TEMP, BATT_OUTPUT_POWER, BATT_OUTPUT_CURRENT, LOAD_VOLTAGE_L1, LOAD_VOLTAGE_L2, 
                              GRID_CURRENT_L1, GRID_CURRENT_L2, INVERTER_OUTPUT_CURRENT_L1, INVERTER_OUTPUT_CURRENT_L2 };

//SoftwareSerial softSerial(MODBUS_RX_PIN, MODBUS_TX_PIN);

ModbusMaster node;


void do_send(osjob_t* j);
void Task1code( void * pvParameters );
void Task2code( void * pvParameters );
TaskHandle_t Task1;
TaskHandle_t Task2;
uint8_t readHoldingRegsResponse[100];
uint8_t data_to_send[35];
uint8_t frame[12] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x64, 0x03, 0x03, 0x20, 0x00, 0x10};
WiFiClient client;
std::vector<uint8_t> data_response;


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
        for(int i = 0; i <=  sizeof(readHoldingRegsResponse)-1; i++)
        {
            printHex2(readHoldingRegsResponse[i]);
        }
        Serial.println();
        
        LMIC_setTxData2(1, readHoldingRegsResponse, sizeof(readHoldingRegsResponse)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void modbusPreTransmission()
{
  delay(500);
  digitalWrite(MODBUS_DIR_PIN, HIGH);
}

void modbusPostTransmission()
{
  digitalWrite(MODBUS_DIR_PIN, LOW);
  delay(500);
}

void setup()
{
  //  esp serial communication
  Serial.begin(115200);
  pinMode(MODBUS_DIR_PIN, OUTPUT);
  digitalWrite(MODBUS_DIR_PIN, LOW);

  //Serial2.begin(baud-rate, protocol, RX pin, TX pin);.
  Serial2.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  Serial2.setTimeout(200);
  //modbus slave ID 14
  node.begin(1, Serial2);
  node.preTransmission(modbusPreTransmission);
  node.postTransmission(modbusPostTransmission);


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
    xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 5, &Task1,  0); 

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


void loop()
{
    uint8_t result;
    uint16_t data[2];
    const int length = sizeof(data_register)/sizeof(data_register[0]);
    int i;
    float reading;
    Serial.print("Loop running on core ");

    Serial.println(xPortGetCoreID());

    esp_task_wdt_init(30, false); 
    result = node.readHoldingRegisters(150, 50);
    Serial.println(result);
    for (int j = 0; j < 50; j++)
      {
          readHoldingRegsResponse[2 * j] = node.getResponseBuffer(j);
          readHoldingRegsResponse[(2 * j) + 1] = node.getResponseBuffer(j) >> 8;
      }
    // if ( node.ku8MBSuccess == node.readHoldingRegisters(150, 10)) {
    //      Serial.println("Success, Received data: ");
    
    // }

    // while(1)
    // {
    //     result = node.readHoldingRegisters(150, 25);
    //     Serial.println(result);
    //     if (result == node.ku8MBSuccess) {
    //       Serial.println("Success, Received data: ");
          
    //       //Retrieve the data from getResponseBuffer(uint8_t u8Index) function.
    //       //that is return 16-bit data. our energy meter return 32-bit data everytime.
    //       //that's why, we take the value to a array called data

    //       for (int j = 0; j < 25; j++)
    //       {
    //         readHoldingRegsResponse[j] = node.getResponseBuffer(j);
    //       }
    //     }
    // }
    
  Serial.println("Testing");
}
