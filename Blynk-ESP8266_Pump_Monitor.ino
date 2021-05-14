#define NAMEandVERSION "PumpMonitor_V0.3"
/*
  This sketch is based on an example posted by Gunner with some modifications added in place to make it able to reconnect after Wifi or Server connection failures.
  It is able to check if it is a Wifi or a server connection issue and recover it when it is possible
  The MCU runs the task every second - It turns the builtin led on and off (allways) and post the millis/1000 to blynk server (only when a connection is available).




    WL_CONNECTED after successful connection is established

    WL_NO_SSID_AVAIL in case configured SSID cannot be reached

    WL_CONNECT_FAILED if connection failed

    WL_CONNECT_WRONG_PASSWORD if password is incorrect

    WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses

    WL_DISCONNECTED if module is not configured in station mode

    -1 on timeout

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
*/

//#define BLYNK_DEBUG
#define BLYNK_TIMEOUT_MS  500  // must be BEFORE BlynkSimpleEsp8266.h doesn't work !!!
#define BLYNK_HEARTBEAT   17   // must be BEFORE BlynkSimpleEsp8266.h works OK as 17s
#define BLYNK_PRINT Serial    
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit ADC version - https://github.com/adafruit/Adafruit_ADS1X15 */


#include "config.h"
#include "DHTesp.h"
DHTesp dht1; 
DHTesp dht2;

BlynkTimer timer;
ComfortState cf;

bool LEDon = 0;
bool online = 0;

bool regen;

float h1;
float t1;

float h2;
float t2;

float  Pressure;
bool pumpStatus;

signed short int rssi = 0;
signed short int signalQuality = 0;

#define DHT22PIN1 12 //D7 - ON THE PIPE
#define DHT22PIN2 13 //D6 - AIR

#define RelayPin 14  // D5


#define wifiSignalVPin V4
#define tempVPin1 V2  //air
#define humVPin1 V1 //air
#define tempVPin2 V12 // pipe
#define humVPin2 V11 // pipe

bool RegenTimer;
bool Geofence;

#define PressureVPin V5
#define RelayVPin V6
#define pumpStatusVPin V7

bool PmpCMD = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(RelayPin, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  
  Serial.begin(115200);
  Serial.println();
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.begin();
  WiFi.hostname(NAMEandVERSION);
  WiFi.mode(WIFI_STA);
  Blynk.config(auth, serveraddr, port);  // I am using the local Server
  CheckConnection();// It needs to run first to initiate the connection.Same function works for checking the connection!
  dht1.setup(DHT22PIN1, DHTesp::DHT22); // Connect DHT sensor to GPIO 17 - D5
  dht2.setup(DHT22PIN2, DHTesp::DHT22);
  timer.setInterval(5000L, CheckConnection); 
  timer.setInterval(1000L, myTimerEvent);     
  timer.setInterval(1550L, PeriodicSync);  
  //timer.setInterval(500L, pumpState);
  
}

void loop() {
  if(Blynk.connected()){
    Blynk.run();
  }
  timer.run();
}


void CheckConnection(){    // check every 11s if connected to Blynk server
  if(!Blynk.connected()){
    online = 0;
    //yield();
    //Serial.println(WiFi.status());
    //if (WiFi.status() != WL_CONNECTED) -- apparently does not recognise when the mcu is alread connected
    if (!WiFi.isConnected())
    {
      Serial.println("Not connected to Wifi! Connect...");
      //Blynk.connectWiFi(ssid, pass); // used with Blynk.connect() in place of Blynk.begin(auth, ssid, pass, server, port);
     // WiFi.config(arduino_ip, gateway_ip, subnet_mask);
      Serial.println(WiFi.status());
      WiFi.begin(ssid, pass);
      delay(400); //give it some time to connect
      //Serial.println(WiFi.status());
      //if (WiFi.status() != WL_CONNECTED)
      if (!WiFi.isConnected())
      {
        Serial.println("Cannot connect to WIFI!");
        Serial.println(WiFi.status());
        online = 0;
      }
      else
      {
        Serial.println("Connected to WiFi!");
        Serial.println(WiFi.localIP());
        rssi = WiFi.RSSI();
        signalQuality = ((1 - ( (-30) - float(rssi) ) / 70) * 100) ;
        Serial.print("Signal Quality: ");
        Serial.println(signalQuality);
      }
    }
    
    if ( WiFi.status() == WL_CONNECTED && !Blynk.connected() )
    {
      Serial.println("Not connected to Blynk Server! Connecting..."); 
      Blynk.connect();  // // It has 3 attempts of the defined BLYNK_TIMEOUT_MS to connect to the server, otherwise it goes to the enxt line 
      if(!Blynk.connected()){
        Serial.println("Connection failed!"); 
        online = 0;
      }
      else
      {
        online = 1;
      }
    }
  }
  else{
    Serial.println("Connected to Blynk server!"); 
    Serial.print("Ip address: ");
    Serial.println(WiFi.localIP());
    rssi = WiFi.RSSI();
    signalQuality = ((1 - ( (-30) - float(rssi) ) / 70) * 100) ;
    Serial.print("Signal Quality: ");
    Serial.print(signalQuality);
    Serial.println("%");
    online = 1;    
  }
}


void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  if (online == 1)
  {
   //Blynk.virtualWrite(int16_t30, millis() / 1000);    
  }
  else 
  {
   // Serial.println("Working Offline!");  
  }
  
  if (LEDon == 0)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)    
    LEDon = 1;
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW                  
    LEDon = 0;
  }
  Serial.println(millis() / 1000);
}

BLYNK_WRITE(V6) 
{
  PmpCMD = param.asInt(); // assigning incoming value from pin V1 to a variable
  if (PmpCMD == 1) {
    Serial.println("PUMP is ON");  
    digitalWrite(RelayPin, HIGH);   // turn the   
  }
  else{
    Serial.println("PUMP is OFF");  
    digitalWrite(RelayPin, LOW);   // turn the L  
  }
}

void pumpSTOPcmd() {
  if (PmpCMD == 1) {
    Serial.println("Floor 1 Light is ON");  
    digitalWrite(RelayPin, HIGH);   // turn the LED stripe on    
  }
  else{
    Serial.println("Floor 1 Light is OFF");  
    digitalWrite(RelayPin, LOW);   // turn the LED stripe on    
  }
}

BLYNK_WRITE(V20)
{
  RegenTimer = param.asInt();
}

BLYNK_WRITE(V21)
{
  int GeofenceTemp = param.asInt(); // Geofence value does not allways get stored on the server
  Blynk.virtualWrite(V22, GeofenceTemp);
}

BLYNK_WRITE(V22)
{
  Geofence = param.asInt(); // Geofence value does not allways get stored on the server
}

bool getTemperatureAIR() {
  yield();
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht1.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht1.getStatus() != 0) {
    Serial.println("DHT22 AIR error status: " + String(dht1.getStatusString()));
    return false;
  }

  float heatIndex = dht1.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht1.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dht1.getComfortRatio(cf, newValues.temperature, newValues.humidity);
  //  h = dht.getHumidity();
  //  t = dht.getTemperature(); // or dht.readTemperature(true) for Fahrenheit
  h1 = newValues.humidity;
  t1 = newValues.temperature;
  Serial.println("T(air)= " + String(t1));
  Serial.println("H(air)= " + String(h1));
  yield();
}

bool getTemperaturePIPE() {
  yield();
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht2.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht2.getStatus() != 0) {
    Serial.println("DHT22 PIPE error status: " + String(dht2.getStatusString()));
    return false;
  }

  float heatIndex = dht2.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht2.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dht2.getComfortRatio(cf, newValues.temperature, newValues.humidity);
  //  h = dht.getHumidity();
  //  t = dht.getTemperature(); // or dht.readTemperature(true) for Fahrenheit
  h2 = newValues.humidity;
  t2 = newValues.temperature;
  Serial.println("T(pipe)= " + String(t2));
  Serial.println("H(pipe)= " + String(h2));
  yield();
}


//
//BLYNK_CONNECTED() {
//
//}

void PeriodicSync()
{
  if(Blynk.connected())
  {
    Serial.println("Periodic Sync...");
    // Push to the server
    getTemperatureAIR();
    if (getTemperatureAIR) {
      Blynk.virtualWrite(V2, t1);
      Blynk.virtualWrite(V1, h1);
    }
    getTemperaturePIPE();
    if (getTemperaturePIPE) {
      Blynk.virtualWrite(V12, t2);
      Blynk.virtualWrite(V11, h2);
    }
    
    Blynk.virtualWrite(V4, signalQuality);
    readPressure();
    pumpState();
    Blynk.virtualWrite(V5, Pressure);   

    if (pumpStatus == 1) {
      Blynk.virtualWrite(V7, 1);   // pump runnig
    }
    else {
      Blynk.virtualWrite(V7, 0);  // pump idle       
    }
    awaynowater();
    if (regen==1){
      Blynk.virtualWrite(V21, 1);  // Water Softener regen      
    }
    else {
      Blynk.virtualWrite(V21, 0);  // Regen is DOONE! 
    }
    Blynk.syncVirtual(V22);
  }
}

void readPressure()
{
  //       VOUT = VCC(0.6667*P+0.1)
  //       A0= 5(0.6667*P+0.1)
  //  A0 = 5*(0.6667*P+0.1) ----- (0.6667*P+0.1) = (A0/5)    ------- (0.6667*P) + 0.1 = (A0/5)  ------- 0.6667*P = [(A/5)-0.1]  --------------- P =  [(A/5)-0.1] / 0.6667          

  // P(mpa) = ((A0/5)-0.1) / 0.6667    
  // P(mpa)x 10 = P(bar)
  //AnalogA0 = ads.readADC_Differential_0_1(); 
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //int16_t Voltage = (0.000125 * ads.readADC_SingleEnded(0))*10;
  Serial.println(ads.readADC_SingleEnded(0));
  Pressure = (((0.000125 * ads.readADC_SingleEnded(0))/5)-0.1)/0.6667*10;
  Serial.println("P=" + String(Pressure));
}

void pumpState()
{
  int16_t AnalogA1 = ads.readADC_SingleEnded(1);
  Serial.println("A1"+ String(AnalogA1));
  if (AnalogA1 > 8000) {
    pumpStatus = 0 ; //runninng 
  }
  else{
    pumpStatus = 1 ;  //idle
  }

}

void awaynowater()
{
  if (pumpStatus == 1 ){
    //Serial.println("Pump is RUnning");
  }
  else{
  }

  if (RegenTimer == 1 && Blynk.connected()){ // the regen interval set in the timer widget
    if (pumpStatus == 1) {
      regen==1;
      //Serial.println("Regen Time");
      //Blynk.notify("The water softener is regenerating");
           
    }
    else {
      regen == 0;
      //Blynk.notify("Regenerating is DONE.");     
    }

  }
  else if (RegenTimer == 0 && Blynk.connected()){
    //Serial.println("Pump is NOT runnning");
  }

  if (Geofence == 0 ){
    if (pumpStatus == 1 ){
      //Serial.println("Pump is Running.You are not home!");
      Blynk.notify("Pump is Running.You are not home!");
    }
    else{
    }
  }
  else {
    
  }
}
