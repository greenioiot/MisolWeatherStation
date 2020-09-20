//#include <hex.h>
#include <HardwareSerial.h>
const long interval = 16000;  //millisecond
unsigned long previousMillis = 0;
HardwareSerial mySerial(1);
uint8_t cc;
String hexString = "";

int   ii, lenn = 0;

struct misoWeather {
  uint8_t idType;
  uint8_t securityCode;
  float winDir, temp;
  float hum, winSpeed, gustSpeed;
  float accRainfall, uv, light, baroPress;
  String  CRC;
  String checksum;
  float pm2_5;
};

struct pms7003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm01_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms7003data dataPMS;
struct misoWeather dataMiso;
HardwareSerial hwSerial(2);
#define SERIAL1_RXPIN 26
#define SERIAL1_TXPIN 25
unsigned long currentMillis;
void setup() {

  pinMode(33, OUTPUT); // turn on PMS7003
  digitalWrite(33, HIGH); // turn on PMS7003
  delay(400);
  hwSerial.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);



  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, 16, 17);
  previousMillis = millis();
  Serial.println("Start");


}
 
void loop() {


  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    hexString = "";
    Serial.println("recv (HEX): ");

    while (mySerial.available())
    {
      cc = mySerial.read();

      if (cc < 0x10) {
        Serial.print("0");
        hexString +=  String(cc, HEX);
      }
      Serial.print(cc, HEX);
      hexString +=  String(cc, HEX);


    }
    // for testing
    //        hexString = "246665E24D370D0300160000005F42314D018F6AFA";
    //                 24623462B5490C0200010000000000C2EB01891CA6
    Serial.println("");

    //    Serial.println(hexString);
    parseHex(hexString);

    readPMSdata(&hwSerial);

   

    previousMillis = currentMillis;

  }

}

long unsigned int hex2Dec(String hex) {
  String tobeDec = "0x" + hex;
  char str[16];
  tobeDec.toCharArray(str, 16);

  long unsigned int val = strtol(str, NULL, 16);
  //  Serial.println(val);
  return val;
}
void parseHex (String hex) {
  String idType = hex.substring(0, 2);
  String securityCode = hex.substring(2, 4);
  String _winDir  = hex.substring(4, 7);
  String _temp  = hex.substring(7, 10);
  String _hum = hex.substring(10, 12);
  String _winSpeed = hex.substring(12, 14);
  String _gustSpeed = hex.substring(14, 16);
  String _accRainfall = hex.substring(16, 20);
  String _uv = hex.substring(20, 24);
  String _light = hex.substring(24, 30);
  String _baroPress = hex.substring(35, 40);
  dataMiso.winDir = hex2Dec(_winDir);
  dataMiso.temp =  (hex2Dec(_temp) - 400)  / 10.0;
  dataMiso.hum = hex2Dec(_hum);
  dataMiso.winSpeed = (hex2Dec(_winSpeed) / 8) * 1.12;
  dataMiso.gustSpeed = hex2Dec(_gustSpeed);
  dataMiso.accRainfall = hex2Dec(_accRainfall);
  dataMiso.uv = hex2Dec(_uv);
  dataMiso.light = hex2Dec(_light) / 10.0;
  dataMiso.baroPress = hex2Dec(_baroPress) / 100.0;
  Serial.println("Debug:");
  Serial.print("idType:");  Serial.println(dataMiso.idType);
  Serial.print("securityCode:");  Serial.println(dataMiso.securityCode);
  Serial.print("winDir:"); Serial.println(dataMiso.winDir);
  Serial.print("temp:");  Serial.println(dataMiso.temp);
  Serial.print("hum:");   Serial.println(dataMiso.hum);
  Serial.print("winSpeed:");   Serial.println(dataMiso.winSpeed);
  Serial.print("gustSpeed:");   Serial.println(dataMiso.gustSpeed);
  Serial.print("accRainfall:");   Serial.println(dataMiso.accRainfall);
  Serial.print("uv:");   Serial.println(dataMiso.uv);
  Serial.print("light:");   Serial.println(dataMiso.light);
  Serial.print("baroPress:");   Serial.println(dataMiso.baroPress);
  Serial.print("pm2.5:");  Serial.println(dataPMS.pm25_env);
  Serial.print("pm10:");  Serial.println(dataPMS.pm100_env);
  Serial.println("end");


}

boolean readPMSdata(Stream *s) {
  Serial.println("readPMSdata");
  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  memcpy((void *)&dataPMS, (void *)buffer_u16, 30);
  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
  if (sum != dataPMS.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

//246220E2B04E0B0400000000000000A439018A0893
