//----------------------------------------------------------------------------------------------------------------------
// TinyTX - An ATtiny84 and RFM12B Wireless Temperature & Humidity Sensor Node
// By Nathan Chantrell. For hardware design see http://nathan.chantrell.net/tinytx
//
// Using the DHT22 temperature and humidity sensor
//
// Licenced under the Creative Commons Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0) licence:
// http://creativecommons.org/licenses/by-sa/3.0/
//
// Requires Arduino IDE with arduino-tiny core: http://code.google.com/p/arduino-tiny/
//----------------------------------------------------------------------------------------------------------------------

#define RF69_COMPAT 1
#include <DHT22.h> // https://github.com/nathanchantrell/Arduino-DHT22
#include <JeeLib.h> // https://github.com/jcw/jeelib
#include <TinyDebugSerial.h>

ISR(WDT_vect) {
  Sleepy::watchdogEvent();  // interrupt handler for JeeLabs Sleepy power saving
}

#define myNodeID 4      // RF12 node ID in the range 1-30
#define network 210      // RF12 Network group
#define freq RF12_433MHZ // Frequency of RFM12B module

#define USE_ACK           // Enable ACKs, comment out to disable
#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 10       // Number of milliseconds to wait for an ack

#define DHT22_PIN 9      // DHT sensor is connected on D9/ATtiny pin 12
#define DHT22_POWER 8    // DHT Power pin is connected on D8/ATtiny pin 11

DHT22 myDHT22(DHT22_PIN); // Setup the DHT

TinyDebugSerial mySerial = TinyDebugSerial();

//########################################################################################################################
//Data Structure to be sent
//########################################################################################################################

typedef struct {
  int humidity;	// Humidity reading
  int supplyV;	// Supply voltage
  int temp;	// Temperature reading
} Payload;

Payload tinytx;
static void showNibble (byte nibble) {
  char c = '0' + (nibble & 0x0F);
  if (c > '9')
    c += 7;
  mySerial.print(c);
}

static void showByte (byte value) {
  showNibble(value >> 4);
  showNibble(value);
  //    Serial.print((word) value);
}

void showPacket () {
  byte n = rf12_len;
  if (rf12_crc == 0)
  {
    mySerial.print("OK ");
  }
  else {
    mySerial.print(" ? ");
    if (n > 20) // print at most 20 bytes if crc is wrong
      n = 20;
  }
  showByte(rf12_grp); mySerial.print(" ");
  showByte(rf12_hdr); mySerial.print(" ");
  showByte(rf12_len);
  for (byte i = 0; i < n + 2; ++i) {
    mySerial.print(" ");
    showByte(rf12_data[i]); 
  }
  mySerial.println("");
}

// Wait a few milliseconds for proper ACK
#ifdef USE_ACK
static byte waitForAck() {
  MilliTimer ackTimer;
  while (!ackTimer.poll(ACK_TIME)) {
    //if (rf12_recvDone() && rf12_crc == 0 &&
    if (rf12_recvDone() && 
        rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID)) {
/*   mySerial.print("rf12_crc = ");
    mySerial.print(rf12_crc);
    mySerial.print(", rf12_hdr = ");
    mySerial.print((word)rf12_hdr);
    mySerial.print(", rf12_hdr_calc = ");
    mySerial.print((word)(RF12_HDR_DST | RF12_HDR_CTL | myNodeID));
    mySerial.println(""); */
   // showPacket();
          return 1;
        }
  }
//  mySerial.println("Fucked");
 // showPacket();
  return 0;
}
#endif

//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//-------------------------------------------------------------------------------------------------
static void rfwrite() {
  byte j=4;
  mySerial.print("temp: ");
  mySerial.print(tinytx.temp);
  mySerial.print(", humidity: ");
  mySerial.print(tinytx.humidity);
  mySerial.print(", volt: ");
  mySerial.println(tinytx.supplyV);

#ifdef USE_ACK
  for (byte i = 0; i <= RETRY_LIMIT; ++i) {  // tx and wait for ack up to RETRY_LIMIT times
    rf12_sleep(-1);              // Wake up RF module
    while (!rf12_canSend() && j-->0)
      rf12_recvDone();
    if (j == 0) {
      mySerial.println("canSend() failed");
      return;
    }
    rf12_sendStart(RF12_HDR_ACK, &tinytx, sizeof tinytx);
    rf12_sendWait(2);    // Wait for RF to finish sending while in standby mode
    byte acked = waitForAck();  // Wait for ACK
    rf12_sleep(0);              // Put RF module to sleep
    if (acked) {
      mySerial.println("got ACK");
      return;  // Return if ACK received
    }
    mySerial.println("missed ACK");
    Sleepy::loseSomeTime(RETRY_PERIOD * 1000);     // If no ack received wait and try again
  }
#else
  rf12_sleep(-1);              // Wake up RF module
  while (!rf12_canSend())
    rf12_recvDone();
  rf12_sendStart(0, &tinytx, sizeof tinytx);
  rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
  rf12_sleep(0);              // Put RF module to sleep
  return;
#endif
}



//--------------------------------------------------------------------------------------------------
// Read current supply voltage
//--------------------------------------------------------------------------------------------------
long readVcc() {
  bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
  long result;
  // Read 1.1V reference against Vcc
#if defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega328
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate Vcc in mV
  ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
  return result;
}

//########################################################################################################################

void setup() {
  mySerial.begin(9600);
  mySerial.println("Starting up...");

  rf12_initialize(myNodeID, freq, network); // Initialize RFM12 with settings defined above
  rf12_sleep(0);                          // Put the RFM12 to sleep
  mySerial.println("  Radio initialized");
  
  pinMode(DHT22_POWER, OUTPUT); // set power pin for DHT to output
  PRR = bit(PRTIM1); // only keep timer 0 going
  mySerial.println("  DHT and interrupt set");
  
  ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
  mySerial.println("Ready.");
}

void loop() {

  digitalWrite(DHT22_POWER, HIGH); // turn DHT sensor on

  DHT22_ERROR_t errorCode;

  Sleepy::loseSomeTime(2000); // Sensor requires minimum 2s warm-up after power-on.

  mySerial.println("Reading sensor...");

  errorCode = myDHT22.readData(); // read data from sensor

  //if (errorCode == DHT_ERROR_NONE) { // data is good

    tinytx.temp = (myDHT22.getTemperatureC() * 100); // Get temperature reading and convert to integer, reversed at receiving end

    tinytx.humidity = (myDHT22.getHumidity() * 100); // Get humidity reading and convert to integer, reversed at receiving end

    tinytx.supplyV = readVcc(); // Get supply voltage

    rfwrite(); // Send data via RF

 // }

  digitalWrite(DHT22_POWER, LOW); // turn DS18B20 off

  mySerial.println("Sleeping...");
  Sleepy::loseSomeTime(60000); //JeeLabs power save function: enter low power mode for 60 seconds (valid range 16-65000 ms)

}

