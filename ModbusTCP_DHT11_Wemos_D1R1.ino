/*
  Usefull Link:
  GPIO: https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
  Library: https://github.com/emelianov/modbus-esp8266
*/

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else //ESP32
#include <WiFi.h>
#endif
#ifndef STASSID
#define STASSID "Experia"
#define STAPSK "12345678"
#endif
#include <ModbusIP_ESP8266.h>

#include <DHT.h>
#define DHTPIN    0     // GPIO0 atau D3
#define DHTTYPE   DHT11 // DHT21
#define LedPin    2     // GPIO2 atau D4

DHT dht(DHTPIN, DHTTYPE);

// current temperature & humidity, updated in loop()
float t = 0.0;
float h = 0.0;

const char* ssid = STASSID;
const char* password = STAPSK;

//Modbus Coil Register
const int coil_reg1 = 0;  //Generated Coil for relay1
const int input_reg1= 0;  //Generated Coil for button1

//Inisial Pin yag digunakan
int button1 = 5;  // D1
int relay1 = 14;  // D5
bool status_button1 = false;

//Inisialisasi Modbus Object
ModbusIP mb;

//millis================================
//Set every  sec read DHT
unsigned long previousMillis = 0; // variable to store the last time the task was run
const long interval = 2000; // time interval in milliseconds (eg 1000ms = 1 second)
//======================================

void Blink(int delayTime)
{
  digitalWrite(LedPin,LOW);
  delay(delayTime);
  digitalWrite(LedPin,HIGH);
  delay(delayTime); 
}

void connectWifi()
{
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to ");
    Serial.println(ssid);
    delay(1000);
  }

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    Blink(100);
    Blink(100);
    delay(1000);
  }
    //Tampilkan Alamat IP pada Serial Monitor jika dapat terhubung
    Serial.print("Connected to");
    Serial.println(ssid);
    Serial.println("Alamat IP: ");
    Serial.println(WiFi.localIP());
    Blink(2000);
}

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(button1, INPUT_PULLUP);
  pinMode(relay1, OUTPUT);
  pinMode(LedPin, OUTPUT);
  
  //Mode Wifi Ke Station
  WiFi.mode(WIFI_STA);
  
  //Konek Wifi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);

   while(WiFi.status() != WL_CONNECTED) {
    Blink(100);
    Blink(100);
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Alamat IP: ");
  Serial.println(WiFi.localIP());
  Blink(2000);
  
  //initial modbus server
  mb.server();

  //configure coil
  mb.addCoil(coil_reg1);    //Generated Coil for relay1
  mb.addIsts(input_reg1);   //Generated Coil for button1

  //configure holding register for temperature & humidity
  mb.addHreg(0);  //for temperatur integer value
  mb.addHreg(1);  //for temperature decimal value
  mb.addHreg(2);  //for humidity integer value
  mb.addHreg(3);  //for humidity decimal value

  //turn off all relay
  digitalWrite(relay1, HIGH);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
  Serial.println("Connection Lost");

    while (WiFi.status() != WL_CONNECTED){
    connectWifi();
    }
  }

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  //=========Nonaktifkan Saat troubleshoot pin input dan output==========\\
  // if (isnan(h) || isnan(t) ) {
  //   Serial.println(F("Failed to read from DHT sensor!"));
  //   return;
  // }
  //=========Nonaktifkan Saat troubleshoot pin input dan output==========\\
  
  //Call once inside loop()
  mb.task();

  //Attach Relay to Coil register
  int coil1   = mb.Coil(coil_reg1);   // Generated Coil for relay1
  int input1  = mb.Ists(input_reg1);  // Generated Coil for button1

  // input_reg1 diganti dengan button1 setelah hadware tombol input terpasang
  // if (button1 == 1) {
  //   mb.Coil(coil_reg1) == 1;
  // }
  // else {
  // mb.Coil(coil_reg1) == 0;
  // }

  unsigned long currentMillis = millis(); // mendapatkan waktu sekarang
  // Checks whether it is time to run the task
  if (currentMillis - previousMillis >= interval) {
    // Save the last time the task was run
    previousMillis = currentMillis;
    // Tulis data sensor pada Holding Register
    int t_int = int(t);
    int t_dec = int((t-t_int) * 100);
    mb.Hreg(0, t_int);
    mb.Hreg(1, t_dec);

    int h_int = int(h);
    int h_dec = int((h - h_int) * 100);
    mb.Hreg(2, h_int);
    mb.Hreg(3, h_dec);

    Serial.print("Humidity:");Serial.print(h); Serial.print("|");
    Serial.print("Temperature:");Serial.print(t);Serial.println();
    Serial.print("CoilReg1:");Serial.print(coil1);Serial.print("|");
    Serial.print("InputReg1:");Serial.print(input1);Serial.print("|");
    Serial.println(digitalRead(button1));
  }

  digitalWrite(LedPin, !coil1);

  //Check button
  check_button();
  
  delay(100);
}

void check_button() {
  //check button1=================================
  int buttonValue1 = digitalRead(button1);
  if (buttonValue1 == LOW )
  {
      if (status_button1 == false)
      {
      //turn on
      status_button1 = true;
      Serial.println("Turn ON Coil1");
      mb.Coil(coil_reg1,1);
      mb.Ists(input_reg1,1);
      } 
  }
  else if (buttonValue1 == HIGH)
  {
      if (status_button1 == true)
      {
      //turn off
      status_button1 = false;
      Serial.println("Turn OFF Coil1");
      mb.Coil(coil_reg1,0);
      mb.Ists(input_reg1,0);
      }
  }
}