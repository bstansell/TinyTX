//----------------------------------------------------------------------------------------------------------------------
// TinyTX Simple Receive Example
// By Nathan Chantrell.
//
// Licenced under the Creative Commons Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0) licence:
// http://creativecommons.org/licenses/by-sa/3.0/
//----------------------------------------------------------------------------------------------------------------------

#define RF69_COMPAT 1
#include <JeeLib.h> // https://github.com/jcw/jeelib
#include <TinyDebugSerial.h>

// Fixed RF12 settings
#define MYNODE 30            //node ID of the receiever
#define freq RF12_433MHZ     //frequency
#define group 210            //network group

TinyDebugSerial mySerial = TinyDebugSerial();

typedef struct {
  int humidity;	// Humidity reading
  int supplyV;	// Supply voltage
  int temp;	// Temperature reading
} Payload;

Payload rx;

int nodeID;    //node ID of tx, extracted from RF datapacket. Not transmitted as part of structure

void setup () {

  mySerial.begin(9600);

  mySerial.println("TinyTX Simple Receive Example");
  mySerial.println("-----------------------------");
  mySerial.println("Initializing radio...");
  rf12_initialize(MYNODE, freq, group); // Initialise the RFM12B
  mySerial.println("Waiting for data");
  mySerial.println(" ");
}

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
  for (byte i = 0; i < n+2; ++i) {
    mySerial.print(" ");
    showByte(rf12_data[i]); 
  }
  mySerial.println("");
}

extern uint8_t mytxbuf[30];
extern uint8_t mytxcnt;
void showSent() {
  mySerial.print("OUT ");
  for (byte i = 0; i < mytxcnt; ++i) {
    mySerial.print(" ");
    showByte(mytxbuf[i]); 
  }
  mySerial.println("");
  mytxcnt = 0;
}
  
void loop() {
  //   if (rf12_recvDone() && (rf12_hdr & RF12_HDR_CTL) == 0) {
  if (rf12_recvDone() && rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0) {
    nodeID = rf12_hdr & 0x1F;  // get node ID
    rx = *(Payload*) rf12_data;
    int value = rx.humidity;
    int temp = rx.temp;
    int millivolts = rx.supplyV;

  showPacket();
    if (RF12_WANTS_ACK) {                  // Send ACK if requested
      rf12_sendStart(RF12_ACK_REPLY, 0, 0);
      mySerial.println("-> ack sent");
      showSent();
    }

  /*
    mySerial.println("Received a packet:");
    mySerial.print("From Node: ");
    mySerial.println(nodeID);
    mySerial.print("Humidity: ");
    mySerial.println(value);
    mySerial.print("Temp: ");
    mySerial.println(temp);
    mySerial.print("TX Millivolts: ");
    mySerial.println(millivolts);
    mySerial.println("--------------------");*/
  }

}
