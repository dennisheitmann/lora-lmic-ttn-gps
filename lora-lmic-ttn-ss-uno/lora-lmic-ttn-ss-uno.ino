/*******************************************************************************

   Arduino Uno SoftwareSerial connection from GPS to Pin 3,4
   2019-08-03 Dennis DO7DH

 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 4); // RX, TX

#include "TinyGPS.h"

TinyGPS gps;

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x00000000; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
// Fuer Dauerbetrieb 120
const unsigned TX_INTERVAL = 30;

char datasend[11];    //Used to store GPS data for uploading
float flat, flon, falt, hdop;

// Pin mapping for Dragino Shield
const lmic_pinmap lmic_pins = {
  .nss = 10,                      // chip select
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,                       // reset pin
  .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
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
      break;
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
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    smartdelay(1000);
    GPSReadandParse();
    if (hdop / 100 < 4.0)
    {
      if (falt < 15000.0)
      {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, datasend, sizeof(datasend) - 1, 0);
        Serial.println(F("Packet queued"));
        Serial.print(F("LMIC.freq:"));
        Serial.println(LMIC.freq);
        Serial.println("");
        Serial.println(F("Receive data:"));
        // Next TX is scheduled after TX_COMPLETE event.
      } else {
        Serial.println(F("Altitude > 15000m, exiting"));
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      }
    } else {
      Serial.println(F("HDOP > 4, exiting"));
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    }
  }
}


void setup() {
  delay(1000);
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(1000);
  Serial.println(F("Starting"));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // disable channels (only use channel 0 - 868.1 MHz)
  // LMIC_disableChannel(0);
  LMIC_disableChannel(1);
  LMIC_disableChannel(2);
  LMIC_disableChannel(3);
  LMIC_disableChannel(4);
  LMIC_disableChannel(5);
  LMIC_disableChannel(6);
  LMIC_disableChannel(7);
  LMIC_disableChannel(8);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC.datarate = DR_SF7;
  LMIC_setDrTxpow(LMIC.datarate, 20);
  LMIC.rps = updr2rps(LMIC.datarate);

  // Start job
  do_send(&sendjob);
}

void GPSReadandParse()
{
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  falt = gps.f_altitude(); //get altitude
  hdop = gps.hdop(); // get hdop

  Serial.println("########GPS########");
  Serial.print("Latitude: ");
  Serial.println(flat, 4);
  Serial.print("Longitude: ");
  Serial.println(flon, 4);
  Serial.print("Altitude: ");
  Serial.println(falt, 0);
  Serial.print("HDOP: ");
  Serial.println(hdop / 100);
  Serial.println("");

  int32_t lat = flat * 10000;
  int32_t lon = flon * 10000;
  int32_t alt = falt;
  int32_t hdo = hdop;

  datasend[0] = lat;
  datasend[1] = lat >> 8;
  datasend[2] = lat >> 16;

  datasend[3] = lon;
  datasend[4] = lon >> 8;
  datasend[5] = lon >> 16;

  datasend[6] = alt;
  datasend[7] = alt >> 8;

  datasend[8] = hdo;
  datasend[9] = hdo >> 8;
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (mySerial.available())
    {
      gps.encode(mySerial.read());
    }
  } while (millis() - start < ms);
}

void loop() {
  if (!(LMIC.opmode & OP_TXRXPEND)) {
    smartdelay(1000);
    GPSReadandParse();
  }
  os_runloop_once();
}
