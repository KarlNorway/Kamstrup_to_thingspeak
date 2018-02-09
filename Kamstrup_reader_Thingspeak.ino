/*
 * Hal9k's Arduino code for reading Kamstrup power meters is used as base for this ESP8266
 * 
 * espsoftwareserial lib can be downlaoded here:
 * https://github.com/plerup/espsoftwareserial
 * 
 */

#include <ESP8266WiFi.h>        // WiFi core lib for ESP8266
#include <espsoftwareserial.h>  // SoftwareSerial lib made for ESP8266
#include <OneWire.h>


//Kamstrup setup
// Kamstrup 382Jx3
word const kregnums[] = { 0x0001,0x03ff,0x0027,0x041e,0x041f,0x0420 };
char* kregstrings[]   = { "Energy in","Current Power","Max Power","Voltage p1","Voltage p2","Voltage p3" };
byte addr[8] = { 0x28, 0x40, 0xB0, 0xBC, 0x02, 0x00, 0x00, 0xE8};

#define NUMREGS 6     // Number of registers above
#define KAMBAUD 9600
#define dsPin 4

OneWire  ds(dsPin);  // WeMos d2 (a 1,7K pullup resistor is necessary)


// Units
char*  units[65] = {"","Wh","kWh","MWh","GWh","j","kj","Mj",
  "Gj","Cal","kCal","Mcal","Gcal","varh","kvarh","Mvarh","Gvarh",
        "VAh","kVAh","MVAh","GVAh","kW","kW","MW","GW","kvar","kvar","Mvar",
        "Gvar","VA","kVA","MVA","GVA","V","A","kV","kA","C","K","l","m3",
        "l/h","m3/h","m3xC","ton","ton/h","h","hh:mm:ss","yy:mm:dd","yyyy:mm:dd",
        "mm:dd","","bar","RTC","ASCII","m3 x 10","ton xr 10","GJ x 10","minutes","Bitfield",
        "s","ms","days","RTC-Q","Datetime"};

// Pin definitions
#define PIN_KAMSER_RX  14  // Kamstrup IR interface RX
#define PIN_KAMSER_TX  12  // Kamstrup IR interface TX
#define PIN_LED        16  // Standard Arduino LED
float kamVal[6];
float celsius;

//Thingspeak Definitions
const char* server = "api.thingspeak.com";
String apiKey = "";  // Write key for your channel

/* 
 *  WiFi definitions
 * Enter in your SSID and Passkey
 */

const char* MY_SSID = "";    
const char* MY_PWD = "";

// Kamstrup optical IR serial
#define KAMTIMEOUT 300  // Kamstrup timeout after transmit
espsoftwareserial kamSer(PIN_KAMSER_RX, PIN_KAMSER_TX, false);  // Initialize serial


// crc_1021 - calculate crc16
long crc_1021(byte const *inmsg, unsigned int len){
  long creg = 0x0000;
  for(unsigned int i = 0; i < len; i++) {
    int mask = 0x80;
    while(mask > 0) {
      creg <<= 1;
      if (inmsg[i] & mask){
        creg |= 1;
      }
      mask>>=1;
      if (creg & 0x10000) {
        creg &= 0xffff;
        creg ^= 0x1021;
      }
    }
  }
  return creg;
}

// kamDecode - decodes received data
float kamDecode(unsigned short const kreg, byte const *msg) {

  // skip if message is not valid
  if (msg[0] != 0x3f or msg[1] != 0x10) {
    return false;
  }
  if (msg[2] != (kregnums[kreg] >> 8) or msg[3] != (kregnums[kreg] & 0xff)) {
    return false;
  }
    
  // decode the mantissa
  long x = 0;
  for (int i = 0; i < msg[5]; i++) {
    x <<= 8;
    x |= msg[i + 7];
  }
  
  // decode the exponent
  int i = msg[6] & 0x3f;
  if (msg[6] & 0x40) {
    i = -i;
  };
  float ifl = pow(10,i);
  if (msg[6] & 0x80) {
    ifl = -ifl;
  }

  // return final value
  return (float )(x * ifl);

}

// kamReceive - receive bytes from Kamstrup meter
unsigned short kamReceive(byte recvmsg[]) {

  byte rxdata[50];  // buffer to hold received data
  unsigned long rxindex = 0;
  unsigned long starttime = millis();
  
  kamSer.flush();  // flush serial buffer - might contain noise

  byte r;
  
  // loop until EOL received or timeout
  while(r != 0x0d){
    
    // handle rx timeout
    if(millis()-starttime > KAMTIMEOUT) {
      Serial.println("Timed out listening for data");
      return 0;
    }

    // handle incoming data
    if (kamSer.available()) {

      // receive byte
      r = kamSer.read();
      if(r != 0x40) {  // don't append if we see the start marker
        // append data
        rxdata[rxindex] = r;
        rxindex++; 
      }

    }
  }

  // remove escape markers from received data
  unsigned short j = 0;
  for (unsigned short i = 0; i < rxindex -1; i++) {
    if (rxdata[i] == 0x1b) {
      byte v = rxdata[i+1] ^ 0xff;
      if (v != 0x06 and v != 0x0d and v != 0x1b and v != 0x40 and v != 0x80){
        Serial.print("Missing escape ");
        Serial.println(v,HEX);
      }
      recvmsg[j] = v;
      i++; // skip
    } else {
      recvmsg[j] = rxdata[i];
    }
    j++;
  }
  
  // check CRC
  if (crc_1021(recvmsg,j)) {
    Serial.println("CRC error: ");
    return 0;
  }
  
  return j;
  
}

// kamSend - send data to Kamstrup meter
void kamSend(byte const *msg, int msgsize) {

  // append checksum bytes to message
  byte newmsg[msgsize+2];
  for (int i = 0; i < msgsize; i++) { newmsg[i] = msg[i]; }
  newmsg[msgsize++] = 0x00;
  newmsg[msgsize++] = 0x00;
  int c = crc_1021(newmsg, msgsize);
  newmsg[msgsize-2] = (c >> 8);
  newmsg[msgsize-1] = c & 0xff;

  // build final transmit message - escape various bytes
  byte txmsg[20] = { 0x80 };   // prefix
  int txsize = 1;
  for (int i = 0; i < msgsize; i++) {
    if (newmsg[i] == 0x06 or newmsg[i] == 0x0d or newmsg[i] == 0x1b or newmsg[i] == 0x40 or newmsg[i] == 0x80) {
      txmsg[txsize++] = 0x1b;
      txmsg[txsize++] = newmsg[i] ^ 0xff;
    } else {
      txmsg[txsize++] = newmsg[i];
    }
  }
  txmsg[txsize++] = 0x0d;  // EOF

  // send to serial interface
  for (int x = 0; x < txsize; x++) {
    kamSer.write(txmsg[x]);
  }

}
/*
 * Connect to WiFi. Runs from setup  
 */

void connectWifi()
{
  Serial.print("Connecting to " + *MY_SSID);
  WiFi.begin(MY_SSID, MY_PWD);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(20);
  }

  Serial.println("");
  Serial.println("Connected");
  Serial.println("");
}//end connect

float checkTemp(byte addr[8]) {

  uint8_t i;
  uint32_t color;
  byte present = 0;
  byte type_s;
  byte data[12];
  float fahrenheit;

  // scan OneWire bus
  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
  }

  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      while (1) // loop forever if no DS182x found
        ;
  }

  // reset OneWire and tell DS182x to acquire temperature
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  // read data from OneWire bus
  for ( i = 0; i < 9; i++)           // we need 9 bytes
    data[i] = ds.read();

  // convert the data to actual temperature
  int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  Serial.println(celsius);

  return celsius;
}


void setup () {
  Serial.begin(115200);

  pinMode(dsPin, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(dsPin, HIGH);
  digitalWrite(PIN_LED, LOW);
  
  // setup kamstrup serial
  pinMode(PIN_KAMSER_RX,INPUT);
  pinMode(PIN_KAMSER_TX,OUTPUT);
  kamSer.begin(KAMBAUD);

  delay(200);

// Connect to your WiFi
  connectWifi(); 
  
  //Serial.println("\n[testKamstrup]");
  // poll the Kamstrup registers for data 
    for (int kreg = 0; kreg < NUMREGS; kreg++) {
      kamReadReg(kreg);
      
      delay(100);
  }
}


// kamReadReg - read a Kamstrup register
float kamReadReg(unsigned short kreg) {

  byte recvmsg[40];  // buffer of bytes to hold the received data
  float rval;        // this will hold the final value

  // prepare message to send and send it
  byte sendmsg[] = { 0x3f, 0x10, 0x01, (kregnums[kreg] >> 8), (kregnums[kreg] & 0xff) };
  kamSend(sendmsg, 5);

  // listen if we get an answer
  unsigned short rxnum = kamReceive(recvmsg);

  // check if number of received bytes > 0 
  if(rxnum != 0){
    
    // decode the received message
    rval = kamDecode(kreg,recvmsg);
    // print out received value to terminal (debug)
    Serial.print(kregstrings[kreg]);
    Serial.print(": ");
    Serial.print(rval);
    Serial.print(" ");
    Serial.println();
   kamVal[kreg] = rval;
   Serial.println("to TS: " + String(kamVal[kreg]));
    return rval;
  }
  
}



/*
 * Send to Thingspeak
 * 
 * values can be:
 *    value1 = Energy in
 *    value2 = current Power
 *    value3 = Max Power
 *    value4 = Vp1
 *    value5 = Vp2
 *    value6 = Vp3
 *    
 *   Thingspeak has eight fields per channel. 
 */

void sendToTS(float value1, float value2, float value3, float value4, float value5)
{
  WiFiClient client;

  if (client.connect(server, 80)) { // use ip 184.106.153.149 or api.thingspeak.com
    Serial.println("WiFi Client connected ");

    String postStr = apiKey;
    postStr += "&field1=";
    postStr += value1;    // Energy in
    postStr += "&field2=";
    postStr += value2;    // Urrent consumption
    postStr += "&field3=";
    postStr += value3;    // V phase 1
    postStr += "&field4=";
    postStr += value4;    // V phase 3
    postStr += "&field5=";
    postStr += value5;    // Temp in C inside fusebox
//    postStr += "&field5=";
//    postStr += value6;
    postStr += "\r\n\r\n";

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);
    delay(100);

  }//end if

  client.stop();
}//end send

void loop () {
  // poll the Kamstrup registers for data 
  
  for (int kreg = 0; kreg < NUMREGS; kreg++) {
    kamReadReg(kreg);
    delay(100);
  }
  
  digitalWrite(PIN_LED, digitalRead(PIN_KAMSER_RX));
  celsius = checkTemp(addr);
 // delay increased to 15 sec as Thingspeak has update restrictions
 sendToTS(kamVal[0], kamVal[1], kamVal[3], kamVal[5], celsius  );
  delay(30000);
  
}

