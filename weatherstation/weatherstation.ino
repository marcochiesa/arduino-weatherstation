/*
  Weather Station

  Initialize an UNO and periodically poll readings from attached sensors, then post to a web service

  Sensors:
  Adafruit HTU21D-F Temperature & Humidity Sensor Breakout Board
  https://www.adafruit.com/products/1899

  Adafruit Ultimate GPS Breakout Board
  https://www.adafruit.com/products/746

  Arduino Ethernet Shield
  https://www.arduino.cc/en/Main/ArduinoEthernetShield

 */

#include <Wire.h>
#include "Adafruit_HTU21DF.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Ethernet.h>

char station[] = "StationX";

Adafruit_HTU21DF htu = Adafruit_HTU21DF();

// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  true

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
char server[] = "http://weatherstationviewer.appspot.com";
EthernetClient client;
// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192, 168, 1, 177);
IPAddress gw(192, 168, 1, 1);

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  Serial.println(F("Weather Station initializing ..."));

  if (!htu.begin()) {
    Serial.println(F("Couldn't find temp and humidity sensor!"));
    while (1);
  }
  Serial.println(F("temp/humidity sensor ready"));

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  Serial.println(F("GPS sensor ready"));

  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Failed to configure Ethernet using DHCP"));
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip, gw);
  }
  Serial.println(F("Ethernet ready"));
  Serial.println(Ethernet.localIP());
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
}

uint32_t timer = millis();

void loop() {
  client.stop();

  String data = F("");
  // if you get a connection, report back via serial:
  if (client.connect(server, 80)) {
    Serial.println(F("reporting conditions to server ..."));
    client.println(F("POST /reportconditions HTTP/1.1"));
    client.println(F("Host: weatherstationviewer.appspot.com"));
    client.println(F("Content-Type: application/x-www-form-urlencoded"));
    client.println(F("Connection: close"));
    client.println(F("User-Agent: Arduino/1.0"));

    data += F("tag="); data += station;
    data += F("&amp;temperature="); data += htu.readTemperature();
    data += F("&amp;humidity="); data += htu.readHumidity();

    // if have satellite fix and sentence is received, check the checksum, parse it...
    // this also sets the newNMEAreceived() flag to false
    if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA()) && GPS.fix) {
        data += F("&amp;latitude="); data += GPS.latitudeDegrees;
        data += F("&amp;longitude="); data += GPS.longitudeDegrees;
    }

    client.print(F("Content-Length: "));
    client.println(data.length());
    client.println();
    client.print(data);
    client.println();
  } else {
    // if you didn't get a connection to the server:
    Serial.println(F("server connection failed"));
  }

  // if there's incoming data from the server print on serial port for debugging.
  if (client.available()) {
    Serial.println(F("response available ..."));
    char c;
    while ((c = client.read()) != -1)
        Serial.write(c);
  }
  Serial.println(F("no further response"));

  delay(10000); //wait for 10 seconds
}
