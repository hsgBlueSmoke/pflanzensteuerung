/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <u8g2_fonts.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiUdp.h>

#include <Time.h>
#include <TimeLib.h>
#include <NTPClient.h>
//Für Höhenberechnung mit Druck
#define SEALEVELPRESSURE_HPA (1013.25)
//OLed Bildschirm
#define OLED_RESET 2  // GPIO0
Adafruit_SSD1306 OLED(OLED_RESET);

/******************************WLAN***************************************/

WiFiClientSecure client;
const char *ssid = "Meins";
const char *password = "BlueSmoke85";

/****************************** Bodenfeuchtesensor Kalibrierung ***************************************/

const int Luftwert = 813;   //getesteter Wert bei Luft
const int Wasserwert  = 447;//getesteter Wert bei Wasser
int Mittelwert = (Luftwert - Wasserwert)/3;    //Mittelwert
int Bodenfeuchte = 0;

/****************************** Sensoren ***************************************/

Adafruit_BME280 bme; //BME initialisieren
BH1750 lightMeter(0x23); // LUXMeter initialisieren

/****************************** NTP ***************************************/

WiFiUDP ntpUDP;
#define NTP_Adresse "europe.pool.ntp.org"
NTPClient timeClient(ntpUDP, NTP_Adresse);

/****************************** Adafruit IO ***************************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  8883
#define AIO_USERNAME    "hsg_BlueSmoke"
#define AIO_KEY         "85213d407be646d69d3454029f0d6efe"
#define MQTT_CONN_KEEPALIVE 70

/****************************** Funktionen ***************************************/

// Store the MQTT server, client ID, username, and password in flash memory.
// This is required for using the Adafruit MQTT library.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
// Set a unique MQTT client ID using the AIO key + the date and time the sketch
// was compiled (so this should be unique across multiple devices for a user,
// alternatively you can manually set this to a GUID or other random value).
const char MQTT_CLIENTID[] PROGMEM  = AIO_KEY __DATE__ __TIME__;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/******************************Feeds***************************************/

const char adafruit_feed[] PROGMEM = AIO_USERNAME "/feeds/testumgebung/";
//const char Adafruit_Sub[] PROGMEM = AIO_USERNAME "/feeds/testumgebung.led";

Adafruit_MQTT_Publish TemperaturStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.temperatur");
Adafruit_MQTT_Publish LuftfeuchtigkeitStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.luftfeuchtigkeit");
Adafruit_MQTT_Publish DruckStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.druck");
Adafruit_MQTT_Publish LuxStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.lux");
Adafruit_MQTT_Publish LichtfehlerStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.lichtfehler");
Adafruit_MQTT_Publish GrowStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.grow");
Adafruit_MQTT_Publish BodenStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.boden");
//Adafruit_MQTT_Subscribe LampenStream = Adafruit_MQTT_Subscribe(&mqtt, Adafruit_Sub);

/******************************Eingänge/Ausgänge***************************************/

const int AdresseI2C1 = D1;
const int AdresseI2C2 = D2;

const int AnalogAdresse = A0;
const int AdresseWasser = D0;
const int AdresseLicht = D5;
const int AdresseWind = D6;
const int AdresseAbluft = D7;
const int AdresseNebel = D8;

const int AdresseTaster = D3;

//Ausgänge; HIGH = AUS

bool A_Wasser = HIGH;
bool A_Wind = HIGH;
bool A_Licht = HIGH;
bool A_Abluft = HIGH;
bool A_Nebel = HIGH;

/******************************Merker***************************************/

bool Grow = HIGH;
bool TasterMerker = HIGH;
bool Taster = LOW;
bool Lichtfehler = false;

/******************************Variablen***************************************/

float Temperatur = 0;
int Luftfeuchtigkeit = 0;
int Druck = 0;
int Hoehe = 0;
uint16_t helligkeit = 0;
int BodenfeuchteBerechnet = 0;

String WasserPrint = "?";
String DatumZeit;

const int setTimeOffset = 5000L;

//WiFiServer server(80);

/******************************Zeiten***************************************/

unsigned long VergangeneSendeZeit = 0;
unsigned long Sendeinterval = 60000;

unsigned long VergangeneWindZeit  = 0;
unsigned long WindintervalAus     = 600000;      //10min. aus 600000
unsigned long WindintervalAn      = 300000;      //5min. an 300000

unsigned long VergangeneNebelZeit   = 0;
unsigned long NebelintervalAn       = 5000;       //5 sek. an
unsigned long NebelintervalAus      = 20000;    //5 min. warten 300000

unsigned long VergangeneWasserZeit  = 0;
unsigned long WasserintervalAn      = 5000;    //10 sek. an 10000
unsigned long WasserintervalAus     = 10000;   //5 min. warten 300000

int LichtZeit = 0;
int LichtStart = 0;
int ntpStunden = 0;
String PflanzenPhase = "Grow";
const char* PflanzenPhase_char = "Grow Phase";
const char* Lichtfehler_char = "Lichtüberwachung OK";

/******************************Setup***************************************/

void setup() {

    Serial.begin(115200);
    delay(5000);
    Serial.println(F("Pflanzensteuerung"));
    Serial.println();
    Serial.println("******************************");

    Wire.begin(AdresseI2C1, AdresseI2C2); //Benutzte GPIOs I2C für I2C
    Wire.setClock(100000);
    
    bool status;

    OLED.begin();
    OLED.clearDisplay();

    OLED.setTextWrap(false);
    OLED.setTextSize(1);
    OLED.setTextColor(WHITE);
    OLED.setCursor(0,0);

    OLED.println("    Stefftec");
    OLED.display();
    delay(2000);

    OLED.println();
    OLED.println("Pflanzensteuerung 1.0");
    OLED.display();

    delay(2000);
    OLED.clearDisplay();
    OLED.setCursor(0,0);

    // BME wird initialisiert
     status = bme.begin();
    if (!status) {
        Serial.println("BME280 nicht gefunden");
        OLED.println("BME280: Fehler");
        OLED.display();
        while (1);
    }
     else {
      Serial.println("BME280 gefunden");
      OLED.println("BME280: OK");
      OLED.display();
     }

    //BH initialisiert
    lightMeter.begin();

    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
      Serial.println("BH1750 gefunden");
      OLED.println("BH1750: OK");
      OLED.display();
    }
    else {
      Serial.println("BH1750 nicht gefunden");
      OLED.println("BH1750:Fehler");
      OLED.display();
    }

    WiFi.begin(ssid, password);
    if (!WiFi.status()) {
      Serial.println("Wlan: verbinde");
      OLED.println("Wlan: verbinde");
      OLED.display();
      while (1);
    }
    else {
      Serial.println("Wlan:    OK");
      OLED.print("Wlan:   OK ");
      OLED.display();
    }
    
    Serial.print("Wlan ");
    long rssi = WiFi.RSSI();
    Serial.print("RSSI:");
    Serial.println(rssi);
    //OLED.print("Wlan:   ");
    OLED.print(rssi);
    OLED.println(" dB");
    OLED.print("IP: ");
    OLED.println(WiFi.localIP());
    OLED.display();

    //server.begin();
    Serial.print("lokale IP: ");
    Serial.println(WiFi.localIP());

    timeClient.begin();
    timeClient.setTimeOffset(7200);   //Uhrzeit von UTC auf Lokalzeit 7200
    timeClient.update();

    void connect();
    //Empfange Feed
    //mqtt.subscribe(&LampenStream);

    Serial.println();

    pinMode(AdresseWasser, OUTPUT);
    pinMode(AdresseWind, OUTPUT);
    pinMode(AdresseLicht, OUTPUT);
    pinMode(AdresseAbluft, OUTPUT);
    pinMode(AdresseNebel, OUTPUT);

    pinMode(AdresseTaster, INPUT);

    digitalWrite(AdresseWasser, A_Wasser);
    digitalWrite(AdresseWind, A_Wind);
    digitalWrite(AdresseLicht, A_Licht);
    digitalWrite(AdresseAbluft, A_Abluft);
    digitalWrite(AdresseNebel, A_Nebel);

    delay(500);

    OLED.clearDisplay();

}



/******************************Programm***************************************/


void loop() {

    //connect();
    //AdafruitIO();
    Zeit();
    Lesen();
    //Empfange();
    Programm();
    Ausgaenge();
    SerielleAusgabe();
    Display();
    Sende();
    delay(50);

}


/*  MQTT neu in Void Sende
void AdafruitIO() {

  Adafruit_MQTT_Subscribe *Subscribe;

   if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }

  //Befehle Empfangen

  while (subscription = mqtt.readSubscription(1000)) {

    // we only care about the lamp events
    if (subscription == &LampenStream) {

      // convert mqtt ascii payload to int
      char *value = (char *)LampenStream.lastread;
      Serial.print(F("Received: "));
      Serial.println(value);

      // Apply message to lamp
      String message = String(value);
      message.trim();
      if (message == "ON") {digitalWrite(D4, HIGH);}
      if (message == "OFF") {digitalWrite(D4, LOW);}




    }
  }

  delay(20000);
}
*/

/*
void Empfange() {

    if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
    }

  Adafruit_MQTT_Subscribe *subscription;


  while (subscription = mqtt.readSubscription(50)) {

    // we only care about the lamp events
    if (subscription == &LampenStream) {

      Serial.print("Empfange: ");
      Serial.println((char *)LampenStream.lastread);

      if (strcmp((char *)LampenStream.lastread, "ON") == 0) {
        Grow = HIGH;
      }
      if (strcmp((char *)LampenStream.lastread, "OFF") == 0) {
        Grow = LOW;
      }
    }
  }
}
*/
void Zeit() {

  timeClient.update();
  ntpStunden = timeClient.getHours(); 
}

void Lesen() {


    Temperatur = (bme.readTemperature());

    Druck = (bme.readPressure() / 100.0F);

    Hoehe = (bme.readAltitude(SEALEVELPRESSURE_HPA));

    Luftfeuchtigkeit = (bme.readHumidity());

    helligkeit = lightMeter.readLightLevel();

    Bodenfeuchte = analogRead(AnalogAdresse);
    BodenfeuchteBerechnet = ((Bodenfeuchte - Luftwert) / (-3.66)); //Berechneter Wert in %
    if (BodenfeuchteBerechnet > 100){
      BodenfeuchteBerechnet = 100;
    }
    if (BodenfeuchteBerechnet <= 0){
      BodenfeuchteBerechnet = 0;
    }
}



void Sende() {

  if(millis() - VergangeneSendeZeit > Sendeinterval) {
    VergangeneSendeZeit = millis(); // aktuelle Zeit abspeichern
    //Empfange();

    //if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected()) {
      connect();
  }

  /*Adafruit_MQTT_Subscribe *subscription;

  while (subscription = mqtt.readSubscription(10000)) {

    // we only care about the lamp events
    if (subscription == &LampenStream) {

      Serial.print("Empfange: ");
      Serial.println((char *)LampenStream.lastread);
      */
      /*convert mqtt ascii payload to int
      char *LampenZustand = (char *)LampenStream.lastread;
      Serial.print(F("Empfange: "));
      Serial.println(LampenZustand);

      // Apply message to lamp
      String message = String(LampenZustand);
      message.trim();
      if (message == "ON") {digitalWrite(D5, HIGH);}
      if (message == "OFF") {digitalWrite(D5, LOW);}
    else {
      Serial.println("nix zu empfangen");

    }
  }
    */

  /*Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    // Check if its the onoff button feed
    if (subscription == &LampenStream) {
      Serial.print(F("On-Off button: "));
      Serial.println((char *)LampenStream.lastread);
      
      if (strcmp((char *)LampenStream.lastread, "ON") == 0) {
        digitalWrite(AdresseAbluft, LOW); 
      }
      if (strcmp((char *)LampenStream.lastread, "OFF") == 0) {
        digitalWrite(AdresseAbluft, HIGH); 
      }
    }
  }
  */
    // Publish data
  if (! TemperaturStream.publish(Temperatur))
    Serial.println(F("Failed to publish temperatur"));
  else
    Serial.print(timeClient.getFullFormattedTime());
    Serial.print(F(" Temperatur gesendet: "));
    Serial.println(Temperatur);

  if (! LuftfeuchtigkeitStream.publish(Luftfeuchtigkeit))
    Serial.println(F(" Failed to publish Luftfeuchtigkeit"));
  else
    Serial.print(timeClient.getFullFormattedTime());
    Serial.print(F(" Luftfeuchtigkeit gesendet: "));
    Serial.println(Luftfeuchtigkeit);

  if (! DruckStream.publish(Druck))
    Serial.println(F(" Failed to publish Druck"));
  else
    Serial.print(timeClient.getFullFormattedTime());
    Serial.print(F(" Druck gesendet: "));
    Serial.println(Druck);

  if (! LuxStream.publish(helligkeit))
    Serial.println(F(" Failed to publish Lux"));
  else
    Serial.print(timeClient.getFullFormattedTime());
    Serial.print(F(" Lux gesendet: "));
    Serial.println(helligkeit);

  if (! GrowStream.publish(PflanzenPhase_char))
    Serial.println(F(" Failed to publish GrowStatus"));
  else
    Serial.print(timeClient.getFullFormattedTime());
    Serial.print(F(" GrowStatus: "));
    Serial.println(PflanzenPhase_char);
    
  if (! BodenStream.publish(BodenfeuchteBerechnet))
    Serial.println(F(" Failed to publish Bodenfeuchte"));
  else
    Serial.print(timeClient.getFullFormattedTime());
    Serial.print(F(" Bodenfeuchte: "));
    Serial.println(BodenfeuchteBerechnet);
    
  if (! GrowStream.publish(Lichtfehler_char))
    Serial.println(F(" Failed to publish Bodenfeuchte"));
  else
    Serial.print(timeClient.getFullFormattedTime());
    Serial.println(Lichtfehler_char);
    }
}

//Verbindung zu IO
void connect() {

  OLED.clearDisplay();
  OLED.setTextWrap(false);
  OLED.setTextSize(1);
  //OLED.setTextColor(WHITE);
  OLED.setCursor(0,0);
  //OLED.println("verbinde... ");
  //OLED.display();
  Serial.print(F("Connecting to Adafruit IO... "));
  //OLED.println("Verbinde zu Cloud");
  //OLED.display();

  int8_t ret;

  while ((ret = mqtt.connect()) != 0) {

    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); OLED.print("Wrong protocol"); break;
      case 2: Serial.println(F("ID rejected")); OLED.print("ID rejected"); break;
      case 3: Serial.println(F("Server unavail")); OLED.print("Server unavail"); break;
      case 4: Serial.println(F("Bad user/pass")); OLED.print("Bad user/pass"); break;
      case 5: Serial.println(F("Not authed")); OLED.print("Not authed"); break;
      case 6: Serial.println(F("Failed to subscribe")); OLED.print("Failed to subscribe"); break;;
      default: Serial.println(F("Connection failed")); OLED.print("Connection failed"); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    //OLED.clearDisplay();
    //OLED.println("Verbinde erneut");
    //OLED.display();

  }

  Serial.println(F("Adafruit IO Connected!"));
  //OLED.clearDisplay();
  //OLED.println("Verbindung erfolgreich");
  //OLED.display();

}



void SerielleAusgabe() {



    Serial.println(timeClient.getFullFormattedTime());
    //Serial.println(timeClient.getEpochTime());

    Serial.print("Temperatur = ");
    Serial.print(Temperatur);
    Serial.println(" *C ");

    Serial.print("Druck = ");
    Serial.print(Druck);
    Serial.println(" hPa ");

    Serial.print("Höhe = ");
    Serial.print(Hoehe);
    Serial.println(" m ");

    Serial.print("Luftfeuchtigkeit = ");
    Serial.print(Luftfeuchtigkeit);
    Serial.println(" % ");

    Serial.print("Analogwert = ");
    Serial.print(Bodenfeuchte / 10.23);
    Serial.println(" %");

    Serial.print("Wasserpumpe = ");
    Serial.println(WasserPrint);

    Serial.print("Lichtstärke: ");
    Serial.print(helligkeit);
    Serial.println(" lx");

}

void Programm() {

/******************************Taster Grow***************************************/

  Taster = digitalRead(AdresseTaster);
  if (Taster == HIGH && TasterMerker == LOW) {
    if (Grow == HIGH)
      Grow = LOW;
    else
      Grow = HIGH;
  }
  TasterMerker = Taster;

/******************************Licht***************************************/

  if (Grow == HIGH) {         //Wenn Pflanze im Grow Stadium dann 10h Licht
    LichtZeit = 18;
    LichtStart = 5;
    PflanzenPhase = "Grow";
    PflanzenPhase_char = "Grow Phase";
  }
  else {
    LichtZeit = 6;     //Wenn Pflanze im Ernte Stadium dann 8h Licht
    LichtStart = 10;
    PflanzenPhase = "Bloom";
    PflanzenPhase_char = "Bloom Phase";
  }

  if (ntpStunden >= LichtStart) {
    A_Licht = LOW;
  }
  if (ntpStunden >= LichtStart + LichtZeit) {
    A_Licht = HIGH;
  }
  if (A_Licht == LOW && helligkeit < 100) {
    Lichtfehler = true;
    Lichtfehler_char = "!! Geringe Helligkeit festgestellt: Bitte Lampe oder Lichtsensor überprüfen !!";
  }
  else {
  Lichtfehler = false;
  Lichtfehler_char = "Lichtüberwachung OK";
  }

/******************************Bodenfeuchte Ausgang (Wasserpumpe)***************************************/
  
  BodenfeuchteBerechnet = ((Bodenfeuchte - Luftwert) / (-3.66)); //Berechneter Wert in %
  if (BodenfeuchteBerechnet > 100){   //max Wert 100
      BodenfeuchteBerechnet = 100;
  }
  if (BodenfeuchteBerechnet <= 0) {   //min Wert 0
      BodenfeuchteBerechnet = 0;
    }

  if ((millis() - VergangeneWasserZeit) > WasserintervalAus) {
    if (BodenfeuchteBerechnet >= 0 && BodenfeuchteBerechnet <= 60) { //
      A_Wasser = LOW;
      WasserPrint = " AN ";
    }
    else {
      A_Wasser = HIGH;
      WasserPrint = " AUS ";
    if ((millis() - VergangeneNebelZeit) > NebelintervalAus + NebelintervalAn) { 
      A_Wasser = HIGH;
      WasserPrint = " AUS ";
      VergangeneWasserZeit = millis();
      }
    }
  }
  
/******************************Windsimulation***************************************/

  if (millis() - VergangeneWindZeit > WindintervalAus) {
    if (A_Wind == HIGH) {
      A_Wind = LOW;
    }
    if (millis() - VergangeneWindZeit > WindintervalAus + WindintervalAn) {
      A_Wind = HIGH;
      VergangeneWindZeit = millis();
    }
  }

/******************************Nebel/Abluft***************************************/

  if ((millis() - VergangeneNebelZeit) > NebelintervalAus) {
     if (Luftfeuchtigkeit <= 50) {
        A_Nebel = LOW;      
     }        
     else {
        A_Nebel = HIGH;
     }
     if (Luftfeuchtigkeit >= 65) {
      A_Abluft = LOW;
    }
    else {
      A_Abluft = HIGH;
    }
    if ((millis() - VergangeneNebelZeit) > NebelintervalAus + NebelintervalAn) { 
    A_Nebel = HIGH;
    A_Abluft = HIGH;
    VergangeneNebelZeit = millis();
    }
  }
}

void Ausgaenge() {

  digitalWrite(AdresseWasser, A_Wasser);
  digitalWrite(AdresseWind, A_Wind);
  digitalWrite(AdresseLicht, A_Licht);
  digitalWrite(AdresseAbluft, A_Abluft);
  digitalWrite(AdresseNebel, A_Nebel);
}


void Display() {

    OLED.clearDisplay();
    OLED.setTextWrap(false);
    OLED.setTextSize(1);
    //OLED.setTextColor(WHITE);
    OLED.setCursor(0,0);

    OLED.print("T:");
    OLED.print(Temperatur,1);
    OLED.print(" ");
    OLED.print("D:");
    OLED.print(Bodenfeuchte);
    OLED.print(" ");
    OLED.print("L:");
    OLED.print(Luftfeuchtigkeit);
    if (Lichtfehler == true) {
      OLED.print(" F");
    }
    else {
      OLED.print(" N");
    }
    if (Grow == HIGH) {
      OLED.println(" G");
    }
    else {
      OLED.println(" B");
    }
    OLED.print("B:");
    OLED.print(BodenfeuchteBerechnet);

    //OLED.println(" %");
    //OLED.print("");
    //OLED.startscrollleft(0x00, 0x0F); //make display scroll
    /*OLED.print("Helligkeit: ");
    OLED.print(helligkeit);
    OLED.print(" lx");
    */
    //OLED.setCursor(0,10);
    if (A_Wind == HIGH) {
      OLED.print(" Wind in ");
      OLED.print((WindintervalAus - (millis() - VergangeneWindZeit)) * 0.0000167);
      OLED.println(" min");
    }
    if (A_Wind == LOW) {
      OLED.print("Wind noch ");
      OLED.print(((WindintervalAn + WindintervalAus) - (millis() - VergangeneWindZeit)) * 0.0000167);
      OLED.println(" min");
    }
    if (A_Nebel == HIGH) {
      OLED.print("Nebel AUS");
    }
    else {
      OLED.print("Nebel AN");
    }
    if (A_Abluft == HIGH) {
      OLED.println(" Abluft AUS");
    }
    else {
      OLED.println(" Abluft AN");
    }
    if (A_Wind == HIGH) {
      OLED.print("Wind AUS");
    }
    else {
      OLED.print("Wind AN");
    }
    if (A_Licht == HIGH) {
      OLED.print(" Licht AUS");
    }
    else {
      OLED.print(" Licht AN");
    }
    if (mqtt.connected()) {
      OLED.print(" C");
    }
    else {
      OLED.print(" D");
    }
    OLED.display(); //output 'display buffer' to screen
    //OLED.startscrollleft(0x00, 0x0F); //make display scroll
    //OLED.stopscroll();
}
