/*
   This is the code for Vmix Tally using M5StickC based on other simlour projects found on Github
   written by, Ian Henshaw 

         Written on :- 20/1/2020
  */
  
#include "M5StickC.h"
#include "EEPROM.h"
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#define DEBUG 1

WiFiClientSecure secureClient;
WiFiUDP ntpUDP;
WiFiClient espClient;

// Constants
const int SsidMaxLength = 64;
const int PassMaxLength = 64;
const int HostNameMaxLength = 64;
const int TallyNumberMaxValue = 64;

// Settings object
struct Settings
{
  char ssid[SsidMaxLength] = "VodafoneMobileWiFi-025A35";       // your network SSID (name);
  char pass[PassMaxLength] = "1723214510";  // your network key;
  char hostName[HostNameMaxLength] = "192.168.0.101";  // your Vmix ip;
  int tallyNumber = 1;  //vmix Input number to watch


};


Settings settings;
bool apEnabled = false;

// vMix settings
int port = 8099;


// Tally info
char currentState = -1;
const char tallyStateOff = 0;
const char tallyStateProgram = 1;
const char tallyStatePreview = 2;


// The WiFi client
WiFiClient client;
int timeout = 10;
int delayTime = 10000;



// Time measure
int interval = 5000;
unsigned long lastCheck = 0;

uint8_t bat_3[] =
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
  0x00, 0x6e, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdb, 0x00, 0xff,
  0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xdb, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xb7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xdb,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
};

uint8_t bat_2[] =
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
  0x00, 0x6e, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdb, 0x00, 0xff,
  0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xdb, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xb7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xdb,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
};

uint8_t bat_1[] =
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
  0x00, 0x6e, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdb, 0x00, 0xff,
  0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x00, 0xb7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xdb,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
};

void battery_status()
{
  vbat = M5.Axp.GetVbatData() * 1.1 / 1000;
  discharge = M5.Axp.GetIdischargeData() / 2;
  if (vbat >= 4)
  {
    M5.Lcd.pushImage(145, 1, 14, 8, bat_3);
  }
  else if (vbat >= 3.7)
  {
    M5.Lcd.pushImage(145, 1, 14, 8, bat_2);
  }
  else if (vbat < 3.7)
  {
    M5.Lcd.pushImage(145, 1, 14, 8, bat_1);
  }
  else
  {}
  // M5.Lcd.setTextColor(TFT_YELLOW);
  // M5.Lcd.setCursor(140, 12);
  // M5.Lcd.setTextSize(1);
  // M5.Lcd.println(discharge);
}
// Set tally to off
void tallySetOff()
{
  Serial.println("Tally off");
   M5.Lcd.fillScreen(BLACK);
   M5.Lcd.setCursor(7, 25);
   M5.Lcd.setTextSize(5);
   M5.Lcd.setTextColor(WHITE, BLACK);
   M5.Lcd.printf("SAFE");
   digitalWrite(10, HIGH);
   battery_status();
 
}

// Set tally to program
void tallySetProgram()
{
  Serial.println("Tally program");
   M5.Lcd.fillScreen(RED);
   M5.Lcd.setCursor(8, 25);
   M5.Lcd.setTextSize(6);
   M5.Lcd.setTextColor(WHITE, RED);
   M5.Lcd.printf("LIVE");
   digitalWrite(10, LOW);
   battery_status();
}

// Set tally to preview
void tallySetPreview()
{
  Serial.println("Tally preview");
   M5.Lcd.fillScreen(GREEN);
   M5.Lcd.setCursor(5, 25);
   M5.Lcd.setTextSize(5);
   M5.Lcd.setTextColor(BLACK, GREEN);
   M5.Lcd.printf(" PRE");
   digitalWrite(10, HIGH);
   battery_status();
  
}

// Set tally to connecting
void tallySetConnecting()
{ M5.Lcd.fillScreen(BLACK);
 M5.Lcd.setCursor(0, 25);
 M5.Lcd.printf("Connting to Vmix");
}

// Handle incoming data
void handleData(String data)
{
  // Check if server data is tally data
  if (data.indexOf("TALLY") == 0)
  {
    char newState = data.charAt(settings.tallyNumber + 8);

    // Check if tally state has changed
    if (currentState != newState)
    {
      currentState = newState;

      switch (currentState)
      {
        case '0':
          tallySetOff();
          break;
        case '1':
          tallySetProgram();
          break;
        case '2':
          tallySetPreview();
          break;
        default:
          tallySetOff();
      }
    }
  }
  else
  {
    Serial.print("Response from vMix: ");
    Serial.println(data);
  }
}





// Connect to vMix instance
void connectTovMix()
{
  Serial.print("Connecting to vMix on ");
  Serial.print(settings.hostName);
  Serial.print("...");

  if (client.connect(settings.hostName, port))
  {
    Serial.println(" Connected!");
    Serial.println("------------");
    
    tallySetOff();

    // Subscribe to the tally events
    client.println("SUBSCRIBE TALLY");
  }
  else
  {
    Serial.println(" Not found!");
  }
}



void start()
{
  tallySetConnecting();
  
  sprintf(deviceName, "vMix_Tally_%d", settings.tallyNumber);
  sprintf(apPass, "%s%s", deviceName, "_access");

  connectTovMix();
}


void setup()
{
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  if (DEBUG)Serial.begin(115200);
  M5.begin();
  M5.Axp.EnableCoulombcounter();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);

  if (DEBUG)Serial.println(feature);
  pinMode(interruptPin, INPUT_PULLUP);

  // Attempt to connect to Wifi network:
  M5.Lcd.setCursor(25, 25);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.printf("Connecting to wifi");
  M5.Lcd.fillScreen(BLACK);
  if (DEBUG)Serial.print("Connecting Wifi: ");
  if (DEBUG)Serial.println(settings.ssid);
  WiFi.begin(settings.ssid, settings.password);
  while (WiFi.status() != WL_CONNECTED)
  {

    if (DEBUG)Serial.print(".");

  }
  if (DEBUG)Serial.println("");
  if (DEBUG)Serial.println("WiFi connected");
  if (DEBUG)Serial.println("IP address: ");
  IPAddress ip = WiFi.localIP();
  if (DEBUG)Serial.println(ip);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(25, 25);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.printf("Connected");
  
  M5.Lcd.fillScreen(BLACK);

}



 
void loop()
{
  while (client.available())
  {
    String data = client.readStringUntil('\r\n');
    handleData(data);
  }

  if (!client.connected() && !apEnabled && millis() > lastCheck + interval)
  {
    tallySetConnecting();

    client.stop();

    connectTovMix();
    lastCheck = millis();
  }
}
