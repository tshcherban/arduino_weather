#include <SPI.h>
#include "RF24.h"

#include <RestClient.h>
#include <ESP8266WiFi.h>

const char* ssid     = "Keenetic-5911";
const char* password = "D9oNVuw9";
RestClient client = RestClient("192.168.2.15", 5001);

// (7, 8) nano
RF24 myRadio (0, 14);

byte addresses[][6] = {"1Node", "2Node"};

typedef struct
{
  int16_t temp;
  uint16_t humid;
  float pressure;
  uint16_t voltage;
} __attribute__((__packed__)) ClimatReading;

ClimatReading data;

#include <Ticker.h>
Ticker tickerOSWatch;

#define OSWATCH_RESET_TIME 30

static unsigned long last_loop;

void ICACHE_RAM_ATTR osWatch(void) {
  unsigned long t = millis();
  unsigned long last_run = abs(t - last_loop);
  if (last_run >= (OSWATCH_RESET_TIME * 1000)) {
    // save the hit here to eeprom or to rtc memory if needed
    //ESP.restart();  // normal reboot
    Serial.write(27);
    Serial.print("[2B");
    Serial.println("Reset by SW Watchdog...");
    Serial.write(27);
    Serial.print("[2B");
    Serial.flush();
    ESP.reset();  // hard reset
  }
}

void ConnectToWiFi();

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("RF24/Simple Receive data Test"));
  myRadio.begin();
  myRadio.setChannel(108);
  myRadio.setPALevel(RF24_PA_LOW);
  myRadio.setPayloadSize(sizeof(data));
  myRadio.openReadingPipe(1, addresses[0]); // Use the first entry in array 'addresses' (Only 1 right now)
  myRadio.openWritingPipe(addresses[1]);
  myRadio.startListening();
  delay(10);
  Serial.println(F("RF24 setup complete"));

  //ConnectToWiFi();

  Serial.print(F("Starting..."));
  Serial.println(sizeof(data));

  ESP.wdtDisable();

  last_loop = millis();
  tickerOSWatch.attach_ms(((OSWATCH_RESET_TIME / 3) * 1000), osWatch);

  client.setContentType("text/plain");
}//--(end setup )---

unsigned long counter = 0;

String response = "";
char str[50];
byte cmdBytes[sizeof(data)];
bool hasCommand = false;

void ConnectToWiFi()
{
  Serial.println();
  Serial.print(F("Connecting to "));
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }

  Serial.println("");
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
}

void loop()
{
  last_loop = millis();
  ESP.wdtFeed();
  if (myRadio.available()) // Check for incoming data from transmitter
  {
    //Serial.write(27);
    //Serial.print("[2K");

    Serial.print(F("Data available "));
    myRadio.read(&data, sizeof(data));

    if (hasCommand)
    {
      hasCommand = false;

      myRadio.stopListening();
      delay(1);
      myRadio.write(cmdBytes, sizeof(data));
      myRadio.startListening();
      Serial.print(F("Sent command "));
      Serial.println(cmdBytes[0]);
      cmdBytes[0] = 0;
    }

    yield();

    counter++;
    Serial.print(counter);

    ESP.wdtFeed();

    sprintf(str, "/api/data/?value=t:%i;h:%u;p:%f;u:%u", data.temp, data.humid, data.pressure, data.voltage);
    Serial.print(F(" Sending "));
    Serial.println(str);

    yield();
    int statusCode = client.put(str, "");
    Serial.print(F("STATUS: "));
    Serial.println(statusCode);

    ESP.wdtFeed();
    yield();

    String response = "";
    statusCode = client.get("/api/data/command", &response);
    if (statusCode == 200)
    {
      response.trim();
      auto length = response.length();
      response = response.substring(2, length - 2);
      response.trim();

      if (response == "0")
      {
        Serial.println(F("No commands"));
      }
      else
      {
        byte cmd = response.toInt();
        Serial.print(F("Got command "));
        Serial.println(cmd);
        cmdBytes[0] = cmd;
        hasCommand = true;
      }
    }
    else
    {
      Serial.print(F("Unable to get command: HTTP "));
      Serial.println(statusCode);
    }
    return;
    Serial.write(27);
    Serial.print("[2A");

    Serial.write(27);
    Serial.print("[2A");

    // tmp
    Serial.println();
  } //END Radio available

}//--(end main loop )---
