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

#include <BH1750.h> //Lux Sensor

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
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 OLED(OLED_RESET);

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
#define AIO_USERNAME    "IO Benutzername"
#define AIO_KEY         "IO Schlüssel"

/****************************** Funktionen ***************************************/

WiFiClientSecure client;

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

/****************************** Feeds ***************************************/

const char adafruit_feed[] PROGMEM = AIO_USERNAME "/feeds/testumgebung/";
//const char Adafruit_Sub[] PROGMEM = AIO_USERNAME "/feeds/led";

Adafruit_MQTT_Publish TemperaturStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.temperatur");
Adafruit_MQTT_Publish LuftfeuchtigkeitStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.luftfeuchtigkeit");
Adafruit_MQTT_Publish DruckStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.druck");
Adafruit_MQTT_Publish LuxStream = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/testumgebung.lux");
//Adafruit_MQTT_Subscribe LampenStream = Adafruit_MQTT_Subscribe(&mqtt, Adafruit_Sub);

/****************************** Eingänge***************************************/

const int AnalogAdresse = A0; 

//Ausgänge; HIGH = AUS

bool A1_Wasser = HIGH;
bool A2_Nebel = HIGH;
bool A3_Licht = HIGH;
bool A4_Ventilator = HIGH;

/****************************** Konstanten***************************************/

const char *ssid = "WLAN SSID";
const char *password = "WLAN Passwort";


/****************************** Variablen***************************************/

int Analogeingang  = 0;       
int Temperatur = 0;
int Luftfeuchtigkeit = 0;
int Druck = 0;
int Hoehe = 0;
uint16_t helligkeit = 0;


String WasserPrint = "?";
String DatumZeit;

const int setTimeOffset = 5000L;

//WiFiServer server(80);

/****************************** Variablen***************************************/

unsigned long previousMillis = 0;
unsigned long interval = 60000;

/******************************Setup***************************************/

void setup() {
    
    Serial.begin(115200);
    delay(5000);
    Serial.println(F("Pflanzensteuerung 0.1"));
    Serial.println();
    Serial.println("******************************");

    Wire.begin(D1, D2); //Benutzte GPIOs I2C für I2C
    Wire.setClock(100000);
    
    bool status;

    OLED.begin();
    OLED.clearDisplay();
     
    OLED.setTextWrap(false);
    OLED.setTextSize(1);
    OLED.setTextColor(WHITE);
    OLED.setCursor(0,0);

    OLED.println("Pflanzensteuerung 0.1");
    OLED.display();
    

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
    /* Alt
    while ( WiFi.status() != WL_CONNECTED ) {
    Serial.println("Wlan: verbinde");
    OLED.println("Wlan: verbinde");
    OLED.display();
    delay(500);
    }
    */

    Serial.print("Wlan ");
    long rssi = WiFi.RSSI();    
    Serial.print("RSSI:");
    Serial.println(rssi);
    //OLED.print("Wlan:   ");
    OLED.print(rssi);
    OLED.print(" dB");
    OLED.display();

    //server.begin();
    Serial.print("lokale IP: ");
    Serial.println(WiFi.localIP());
    
    timeClient.begin();
    timeClient.setTimeOffset(7200);   //Uhrzeit von UTC auf Lokalzeit 7200 Sommerzeit

    //Empfange Feed
    //mqtt.subscribe(&LampenStream);

    void connect();

    Serial.println();

    pinMode(D4, OUTPUT); // Initialize the LED_BUILTIN pin as an output
    pinMode(D5, OUTPUT);
    pinMode(D6, OUTPUT);
    pinMode(D7, OUTPUT);
    pinMode(D8, OUTPUT);

    digitalWrite(D5, A1_Wasser);
    digitalWrite(D6, A2_Nebel);
    digitalWrite(D7, A3_Licht);
    digitalWrite(D8, A4_Ventilator);
    
    delay(10000);

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

  Adafruit_MQTT_Subscribe *subscription;

  if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }

  
  while (subscription = mqtt.readSubscription(10000)) {

    // we only care about the lamp events
    if (subscription == &LampenStream) {

      Serial.print("Empfange: ");
      Serial.println((char *)LampenStream.lastread);
    }
  }
}
*/
void Zeit() {
  
  timeClient.update();

}

void Lesen() {


    Temperatur = (bme.readTemperature());

    Druck = (bme.readPressure() / 100.0F);

    Hoehe = (bme.readAltitude(SEALEVELPRESSURE_HPA));

    Luftfeuchtigkeit = (bme.readHumidity());
    
    Analogeingang = analogRead (AnalogAdresse); // Analog Values 0 to 1023

    helligkeit = lightMeter.readLightLevel();
    
    Serial.println();

    // Wiederhole alle 50ms
    
}



void Sende() {

  if(millis() - previousMillis > interval) {
    previousMillis = millis();   // aktuelle Zeit abspeichern

    if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
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
    
  
  
    
    Serial.println("warte 60 Sekunden...");

    }

}

//Verbindung zu IO 
void connect() {

  /*OLED.clearDisplay();
  OLED.setTextWrap(false);
  OLED.setTextSize(1);
  //OLED.setTextColor(WHITE);
  OLED.setCursor(0,0);
  //OLED.println("verbinde... ");
  //OLED.display();
  */
  Serial.print(F("Connecting to Adafruit IO... "));

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
    //OLED.println("erneuter Versuch");
    //OLED.display();
    delay(5000);

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
    Serial.print(Analogeingang / 10.23);
    Serial.println(" %");

    Serial.print("Wasserpumpe = ");
    Serial.println(WasserPrint);

    Serial.print("Lichtstärke: ");
    Serial.print(helligkeit);
    Serial.println(" lx");


}

void Programm() {
  
  if (Analogeingang > 50) {                       //Wert noch anpassen Achtung < !!
    A1_Wasser = LOW;
    WasserPrint = " AN ";
  }
  else {
    A1_Wasser = HIGH;
    WasserPrint = " AUS ";
  }

}

void Ausgaenge() {

  digitalWrite(D5, A1_Wasser);
  digitalWrite(D6, A2_Nebel);
  digitalWrite(D7, A3_Licht);
  digitalWrite(D8, A4_Ventilator);
}


void Display() {

    OLED.clearDisplay();
    OLED.setTextWrap(false);
    OLED.setTextSize(1);
    //OLED.setTextColor(WHITE);
    OLED.setCursor(0,0);
    
    OLED.print("Temperatur: ");
    OLED.print(Temperatur);
    OLED.println(" Grad");
    OLED.print("Druck: ");
    OLED.print(Druck);
    OLED.println(" hPa");
    OLED.print("Luftfeuchtigkeit:");
    OLED.print(Luftfeuchtigkeit);
    OLED.println("%");
    OLED.print("Helligkeit: ");
    OLED.print(helligkeit);
    OLED.print(" lx");

    /*if(mqtt.connected()) {
      OLED.print("Verbindungsstatus: verbunden");
    }
    else {
      OLED.print("Verbindungsstatus: getrennt");
    }
    */

    
 
    OLED.display(); //output 'display buffer' to screen  
    //OLED.startscrollleft(0x00, 0x0F); //make display scroll 
}
