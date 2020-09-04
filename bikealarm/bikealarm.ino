/*

      SOFTWARE SERIAl ei toimi jos ei muokkaa sitä librarya sillee että asiat toimii
      (Kiukuttelee, koska pin change interrupt käytössä.)
*/


#include <EEPROM.h>
#include <Wire.h>
//#include <SoftwareSerial.h>
//SoftwareSerial info(7, 8); //rx,tx


String savedData;

int16_t accOffset[3];
int16_t accData[3];
int32_t hyp = 4096; //   =1 g

uint32_t alarmTimer = (uint32_t)~0;
uint32_t timer = 0;
uint8_t alarmCounter = 0;
uint32_t alarmCounterTimer = 0;

float lat, lng;
uint8_t cellAmount;


void setup() {
  CLKPR= 0b10000000; //Clock division 2 
  CLKPR = 0b00000001;
  analogReference(INTERNAL);
  setAsOutput(21);
  //info.begin(38400);
  setAsOutput(2);
  pinLow(2);

  noInterrupts();
  __asm__("wdr");
  WDTCSR |= 0b00011000;
  WDTCSR = 0b01000111; //2s reset interrupt
  interrupts();
  sensorSetup();
  PCICR = 0b00000010;
  PCMSK1 = 0b00000001;

  if(EEPROM[0] == 123){
    EEPROM[0] = 0;
    alarmTimer = 1800000;
  }

  setAsOutput(5);
  setAsOutput(6);
  float temp = voltage();
  temp = voltage();      // first measurement gives wrong readings, don't know why
  //info.println(temp);
  if (temp > 3.85) {
    for (uint8_t i = 0; i < 5; i++) {
      __asm__("wdr");
      pinHigh(6);
      delay(100);
      pinLow(6);
      delay(100);
    }
  }
  else if (temp > 3.7) {
    for (uint8_t i = 0; i < 10; i++) {
      __asm__("wdr");
      pinHigh(6);
      delay(100);
      pinLow(6);
      delay(100);
    }
  }
  else {
    for (uint8_t i = 0; i < 10; i++) {
      __asm__("wdr");
      pinHigh(5);
      delay(100);
      pinLow(5);
      delay(100);
    }
  }
}

void loop() {
  __asm__("wdr");
  timer += 200; //5 hZ
  if (alarmTimer < timer) {
    alarmTimer += 3600000;
    //info.println(alarmTimer);
    ////info.println(timer);
    Serial.begin(9600);
    uint8_t c = 0;
    while (!moduleOn()) {
      c++;
      if (c >= 5) {
        reset();
      }
      Serial.end();
      setAsInput(4);
      pinLow(2);
      for (uint8_t i = 0; i < 10; i++) {
        __asm__("wdr");
        delay(500);
      }
      Serial.begin(9600);
    }
    int16_t success = findGPSData();
    if (success == 0) {
      sendSMS(F("0445746406"), "ALARM\n" + String(lat,6) +" "+ String(lng,6) + "\n" + String(voltage()) + " V\n" + String(cellAmount));
    }
    else {
      sendSMS(F("0445746406"), "ALARM\nNo GPS signal\n" +String(success) +"\n"+ String(voltage()) + " V");
    }
    Serial.end();
    setAsInput(4);
    pinLow(2);
  }
  Wire.beginTransmission(0x68);
  Wire.write(58);
  byte a = Wire.endTransmission();
  if (a) {
    reset();
  }
  Wire.requestFrom(0x68, 1);
  while (!Wire.available()) {}
  uint8_t d = Wire.read();
  a = Wire.endTransmission();
  if (a) {
    reset();
  }
  getAcc();
  int32_t nHyp = sqrt((int32_t)accData[0] * (int32_t)accData[0] + (int32_t)accData[1] * (int32_t)accData[1] + (int32_t)accData[2] * (int32_t)accData[2]);
  hyp = (nHyp * 6 + hyp * 250) / 256;
  if ((abs(nHyp - hyp) > 500) && (timer > 10000) && (alarmTimer == (uint32_t)~0)) {
    if((alarmCounterTimer+10000) < timer){
      alarmCounter = 0;
    }
    if((alarmCounterTimer+500) < timer){
      alarmCounter++;
      if(alarmCounter == 1){
       alarmCounterTimer = timer;
      } 
    }  
    if(alarmCounter >= 3){
      alarmCounter = 0;
      alarmTimer = timer;
    }
  }
  sleepMCU();
}

ISR(PCINT1_vect) {
  SMCR = 0b00000100;
}

ISR(WDT_vect){
  reset();
}

void reset() {
  //info.println("reset");
  if(alarmTimer != (uint32_t)~0){
    EEPROM[0] = 123;
  }
  __asm__("wdr");
  WDTCSR |= 0b00011000;
  WDTCSR = 0b00001000; //0.016s reset time
  while (1) {}
}

void sleepMCU() {
  SMCR = 0b00000101;
  __asm__("sleep");
  SMCR = 0b00000100;
}

void sensorSetup() {
  byte error;
  Wire.begin();
  Wire.setClock((uint32_t)400000);
  PORTC &= B11001111;   //disable internal pullups
  Wire.beginTransmission(0x68);
  Wire.write(107);   //register number
  Wire.write(0b00101001);    //disable sleep mode
  Wire.write(0b01000111);    //cycle 5hz
  error = Wire.endTransmission();
  if (error > 0) {
    reset();
  }
  Wire.beginTransmission(0x68);
  Wire.write(56);   //register number
  Wire.write(0b00000001);    //interrupt
  error = Wire.endTransmission();
  if (error > 0) {
    reset();
  }
  Wire.beginTransmission(0x68);
  Wire.write(25);      //register number
  Wire.write(1);       // sample rate divider (Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) = 1000/(1+1))
  Wire.write(0);  //enable digital low pass filter
  Wire.write(B00011000);   //gyro settings
  Wire.write(B00010000);   // acc settings
  error = Wire.endTransmission();
  if (error > 0) {
    reset();
  }
}

void getAcc() {
  Wire.beginTransmission(0x68);
  Wire.write(59);      //register
  byte a = Wire.endTransmission();
  if (a) {
    reset();
  }
  Wire.requestFrom(0x68, 6);
  while (Wire.available() < 5) {}
  accData[0] = Wire.read() << 8;
  accData[0] |= Wire.read();
  accData[1] = Wire.read() << 8;
  accData[1] |= Wire.read();
  accData[2] = Wire.read() << 8;
  accData[2] |= Wire.read();
  a = Wire.endTransmission();
  if (a) {
    reset();
  }
}

boolean findKeyword(String keyWord, unsigned long howLong, boolean save) {
  unsigned int i = 0;
  savedData = "";
  unsigned long timeOut = millis();
  while (true) {
    __asm__("wdr");
    if (millis() > (timeOut + howLong)) {
      return (false);
    }
    if (Serial.available()) {
      timeOut = millis();
      char ans = Serial.read();
      /* if(Serial.overflow()){
        //info.print("overflow");
        }*/
      //info.print(ans);
      if (save) {
        savedData += ans;
      }

      if (ans == keyWord.charAt(i)) {
        if (i == (keyWord.length() - 1)) {
          if (save) {
            savedData.remove(savedData.indexOf(keyWord)) ;
          }
          return (true);
        }
        else {
          i++;
        }
      }
      else {
        i = 0;
      }
    }
  }
}


boolean moduleOn() {
  pinHigh(2);
  delay(1);
  setAsOutput(4);
  pinLow(4); 
  //info.println("Module ON");
  uint32_t timeOut2 = millis();
  emptyBuffer(10000);
  while (true) {
    if (millis() > (timeOut2 + 30000)) {
      //info.println("Fail");
      delay(1000);
      return (false);
    }
    Serial.print(F("AT\r\n"));
    if (findKeyword("OK", 500, false)) {
      //info.println("jeejee");
      Serial.print(F("AT+CNMI=0,0,0,0\r\n"));
      if (!findKeyword("OK", 5000, false)) {
        return (false);
      }
      Serial.print(F("AT+CMGF=1\r\n"));
      if (!findKeyword("OK", 5000, false)) {
        return (false);
      }
      //info.println("Module ready");
      return (true);
    }
    else {
      //info.println("Error");
    }
  }
}

uint16_t strToInt(String sNum, uint16_t base){
  uint16_t num = 0;
  uint16_t k = 1;
  for(int16_t i=sNum.length()-1; i>=0; i--){
    uint16_t digit = sNum[i];
    if(digit <= 57){    //0,1,2...,9 in ascii
      digit -= 48;
    }
    else{
      digit -= 55;
    }
    num += k*digit;
    k *= base;
  }
  return num;
}

int16_t makeQuery(uint16_t mcc, uint16_t mnc, uint16_t cid, uint16_t lac, float* lat, float* lng){
  Serial.print(F("AT+HTTPDATA=0,1000\r\n"));
  if (!findKeyword("OK", 10000, false)) {
    return -13;
  }
  Serial.print(F("AT+HTTPDATA=130,3000\r\n"));
  if (!findKeyword("DOWNLOAD", 10000, false)) {
    return -14;
  }
  Serial.print(F("{\"radioType\":\"gsm\",\"cellTowers\": [{\"cellId\":"));
  Serial.print(cid);
  Serial.print(F(",\"locationAreaCode\":"));
  Serial.print(lac);
  Serial.print(F(",\"mobileCountryCode\":"));
  Serial.print(mcc);
  Serial.print(F(",\"mobileNetworkCode\":"));
  Serial.print(mnc);
  Serial.print(F("}]}"));
  if (!findKeyword("OK", 10000, false)) {
    return -15;
  }
  Serial.print(F("AT+HTTPACTION=1\r\n"));
  if (!findKeyword("HTTPACTION: 1,", 10000, false)) {
    return -16;
  }
  if (!findKeyword(",", 1000, true)) {
    return -17;
  }
  uint16_t queryStatus = strToInt(savedData,10);
  if(queryStatus != 200){
    return queryStatus;
  }
  if (!findKeyword("\r", 1000, true)) {
    return -18;
  }
  uint16_t dataAmount = strToInt(savedData,10);
  Serial.print(F("AT+HTTPREAD=0,"));
  Serial.print(dataAmount);
  Serial.print(F("\r\n"));
  if (!findKeyword("lat\": ", 10000, false)) {
    return -19;
  }
  if (!findKeyword(",", 1000, true)) {
    return -20;
  }
  *lat = savedData.toFloat();
  if (!findKeyword("lng\": ", 1000, false)) {
    return -21;
  }
  if (!findKeyword("\n", 1000, true)) {
    return -22;
  }
  *lng = savedData.toFloat();
  return 0;
}

int16_t findGPSData() {
  uint16_t mcc[10];
  uint16_t mnc[10];
  uint8_t rxlev[10];
  uint16_t cid[10];
  uint16_t lac[10];
  cellAmount = 0;
  Serial.print(F("AT+CNETSCAN=1\r\n"));
  if (!findKeyword("OK", 5000, false)) {
    return -1;
  }
  Serial.print(F("AT+CNETSCAN\r\n"));
  if (!findKeyword("MCC:", 20000, false)) {
    return -2;
  }
  bool fail = 0;
  while(cellAmount < 10){
    fail = !findKeyword(",MNC:", 1000, true) || fail;
    mcc[cellAmount] = strToInt(savedData,10);
    fail =  !findKeyword(",Rxlev:", 1000, true)|| fail;;
    mnc[cellAmount] = strToInt(savedData,10);
    fail = !findKeyword(",Cellid:", 1000, true)|| fail;;
    rxlev[cellAmount] = (uint8_t)strToInt(savedData,10);
    fail = !findKeyword(",", 1000, true)|| fail;;
    cid[cellAmount] = strToInt(savedData,16);
    fail = !findKeyword("Lac:", 1000, false)|| fail;;
    fail = !findKeyword(",", 1000, true)|| fail;;
    lac[cellAmount] = strToInt(savedData,16);
    if(fail){
      return -3;
    }
    cellAmount++;
    if(!findKeyword("MCC:", 1000, false)){
      break;
    }   
  }
  emptyBuffer(1000);
  Serial.print(F("AT\r\n"));
  if (!findKeyword("OK", 1000, false)) {
    return -4;
  }
  /*for(uint8_t i=0; i<cellAmount; i++){
    //info.print(String(mcc[i]) +" " + String(mnc[i]) +" " + String(rxlev[i]) +" " + String(cid[i]) +" " + String(lac[i])+ "\n" );
  }*/
/*  Serial.print("AT+SAPBR=3,1,\"APN\",internet\r\n");
  if (!findKeyword("OK", 5000, false)) {
    return -5;
  }*/
  bool everythingIsFine = 0;
  for(uint8_t i=0; i<3; i++){
    Serial.print(F("AT+SAPBR=1,1\r\n"));
    if (!findKeyword("OK", 10000, false)) {     //sometimes doesn't work on the first try :)
      emptyBuffer(1000);
    }
    else{
      everythingIsFine = 1;
      break;
    }
  }
  if(!everythingIsFine){
    return -6;
  } 
  Serial.print(F("AT+HTTPINIT\r\n"));
  if (!findKeyword("OK", 5000, false)) {
    return -7;
  }
  Serial.print(F("AT+HTTPSSL=1\r\n"));
  if (!findKeyword("OK", 5000, false)) {
    return -8;
  }
  Serial.print(F("AT+HTTPPARA=\"CID\",1\r\n"));
  if (!findKeyword("OK", 5000, false)) {
    return -9;
  }
  Serial.print(F("AT+HTTPPARA=\"URL\",\"https://www.googleapis.com/geolocation/v1/geolocate?key=YOUR_API_KEY\"\r\n"));
  if (!findKeyword("OK", 5000, false)) {
    return -10;
  }
  lat = 0.0;
  lng = 0.0;
  uint32_t rxlevSum = 0;
  for(uint32_t i=0; i<cellAmount; i++){
    float temp1, temp2;
    int16_t success = makeQuery(mcc[i],mnc[i],cid[i],lac[i],&temp1, &temp2);
    if(success != 0){
      success = makeQuery(mcc[i],mnc[i],cid[i],lac[i],&temp1, &temp2);   //try again
      if(success != 0){
        return success;
      }
    }
    lat += ((float)rxlev[i])*temp1;
    lng += ((float)rxlev[i])*temp2;
    rxlevSum += (uint32_t)rxlev[i];
  }
  emptyBuffer(500);
  lat /= (float)rxlevSum;
  lng /= (float)rxlevSum;
  Serial.print(F("AT+HTTPTERM\r\n"));
  if (!findKeyword("OK", 5000, false)) {
    return -11;
  }
  Serial.print(F("AT+SAPBR=0,1\r\n"));
  if (!findKeyword("OK", 5000, false)) {
    return -12;
  }
  return 0;
}

/*
 * 
 * OLD MOUDLE
boolean moduleOn() {
  pinHigh(2);
  delay(1);
  setAsOutput(4);
  pinLow(4); 
  //info.println("Module ON");
  uint32_t timeOut2 = millis();
  //info.println(timeOut2);
  while (true) {
    //info.println(millis());
    if (millis() > (timeOut2 + 30000)) {
      //info.println("Failllll");
      delay(1000);
      return (false);
    }
    Serial.print("AT\r\n");
    if (findKeyword("OK", 500, false)) {
      //info.println("jeejee");
      if (!findKeyword("+CREG: 1", 15000, false)) {
        return (false);
      }
      Serial.print("AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n");
      if (!findKeyword("OK", 5000, false)) {
        return (false);
      }
      Serial.print("AT+CNMI=0,0,0,0\r\n");
      if (!findKeyword("OK", 5000, false)) {
        return (false);
      }
      Serial.print("AT+CMER=3,0,0,0\r\n");
      if (!findKeyword("OK", 5000, false)) {
        return (false);
      }
      Serial.print("AT+CMGF=1\r\n");
      if (!findKeyword("OK", 5000, false)) {
        return (false);
      }
      Serial.print("AT+GPSRD=4\r\n");
      if (!findKeyword("OK", 5000, false)) {
        return (false);
      }
      //info.println("Module ready");
      return (true);
    }
    else {
      //info.println("Error");
    }
  }
}*/


boolean sendSMS(String number, String message) {
  Serial.print(F("AT+CMGS=\""));
  Serial.print(number);
  Serial.print(F("\"\r\n"));
  if (!findKeyword(">", 5000, false)) {
    return (false);
  }
  Serial.print(message);
  Serial.write(26);
  if (!findKeyword("OK", 5000, false)) {
    return (false);
  }
  Serial.print(F("AT\r\n"));
  if (!findKeyword("OK", 1000, false)) {
    return false;
  }
  return (true);
}


void emptyBuffer(unsigned long howLong) {
  uint32_t timeOut2 = millis();
  while (timeOut2 + howLong > millis()) {
    __asm__("wdr");
    if (Serial.available()) {
      //info.write(Serial.read());
      uint8_t lol = Serial.read();
    }
  }
}

/*
boolean findGPSData() {
  String GPSStatus = "V";
  boolean success = true;
  String GPSData2;
  Serial.print("AT+GPS=1\r\n");
  uint32_t timeOut2 = millis();

  while ((GPSStatus == "V") && (success == true) && (millis() < timeOut2 + 90000)) {
    success = findKeyword("GPRMC,", 10000, false);
    success = findKeyword(",", 1000, false);
    success = findKeyword(",", 1000, true);
    GPSStatus = savedData;
    //info.print("\r\n");
    //info.print("GPSStatus: ");
    //info.println(GPSStatus);
  }
  if ((!success) || (millis() >= timeOut2 + 90000)) {
    Serial.print("AT+GPS=0\r\n");
    emptyBuffer(5000);
    return (false);
  }
  else {
    success = findKeyword(",", 1000, true);
    GPSData2 = savedData;
    success = findKeyword(",", 1000, true);
    GPSData2 += savedData;
    success = findKeyword(",", 1000, true);
    GPSData2 += savedData;
    success = findKeyword(",", 1000, true);
    GPSData2 += savedData;
    if (!success) {
      Serial.print("AT+GPS=0\r\n");
      emptyBuffer(5000);
      return (false);
    }
    else {

      GPSData = GPSData2.substring(0, 2);
      GPSData += " ";
      GPSData += GPSData2.substring(2, 14);
      GPSData += " ";
      GPSData += GPSData2.substring(14);
      Serial.print("AT+GPS=0\r\n");
      emptyBuffer(5000);
      return (true);
    }
  }
}*/

float voltage() {
  pinHigh(21);
  delay(1);
  float temp = analogRead(A2);
  delay(1);
  pinLow(21);
  return ((temp / 1024.0) * 1.07) * 7.8; // voltage dividor: 6800 ohm and 1000 ohm
}

void pinLow(byte pin) {
  if (pin < 8) {
    PORTD &= ~(1 << pin);
  }
  else if (pin > 19) {
    PORTC &= ~(1 << (pin - 20));
  }
  else {
    PORTB &= ~(1 << (pin - 8));
  }
}

void pinHigh(byte pin) {
  if (pin < 8) {
    PORTD |= 1 << pin;
  }
  else if (pin > 19) {
    PORTC |= 1 << (pin - 20);
  }
  else {
    PORTB |= 1 << (pin - 8);
  }
}

void setAsOutput(byte pin) {
  if (pin < 8) {
    DDRD |= 1 << pin;
  }
  else if (pin > 19) {
    DDRC |= 1 << (pin - 20);
  }
  else {
    DDRB |= 1 << (pin - 8);
  }
}

void setAsInput(byte pin) {
  if (pin < 8) {
    DDRD &= ~(1 << pin);
  }
  else if (pin > 19) {
    DDRC &= ~(1 << (pin - 20));
  }
  else {
    DDRB &= ~(1 << (pin - 8));
  }
}
