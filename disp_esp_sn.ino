/*
TODO: Перенести обрабодтчик индекса меню из обрабодтчика прерываний
	  Сделать меню многоуровневым 
	  Добавить автономный режим
	  Добавить подгружаемые с сервера меню для управления системой
	  Добавить поддержку MQTT
	  Рефракторинг кода!!
	  Переработать поддержку RTC и убрать двойную конверсию
	  I2C:
		0x20 - I2C LCD
		0x23 - BH1750
		0x76 - BME280
		0x68 - DS1307
		0x50 - AT24C32 EEPROM
*/

#include "ESP8266HTTPUpdateServer.h"
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#define ESP_CH
#include "config.h"
#include "FS.h"

#include "TimeLib.h"
#include "favicon.c"
#include "a1fl.c" //Библиотека с прекладными функциями
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include "BH1750.h"
#include <ESP8266WiFi.h>


#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"

#include <SPI.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "uRTCLib.h"

#include "LiquidCrystal_I2C.h"
#include <Ticker.h>

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define BUF_SIZE 2512
#define RBUF_SIZE 2048
#define DHT22_PIN 5
#define NAN -200
#define RCOL 30
#define TID 0
#define HID 1

#define USE_SERIAL srlcd

#define OFFSET 10                                           //LCD char offset


#define dataPin 12
#define clockPin 14
#define enablePin 13

#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  

const int fw_ver = 114;

ADC_MODE(ADC_VCC);

Ticker data_collect, data_send_tic;

const char *endl = "\n";
Adafruit_BME280 bme;

uRTCLib rtc(0x68, 0x50);

void buthandleInterrupts();
void POWERBhandleInt();
void print_bool(bool);
void UPBhandleInt();
void DNBhandleInt();



ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;

float mqv=0, mq7=0, mq9=0, vin=0, mc_vcc=0, mc_temp=0, lux=0, esp_vcc=0, tmp=0, mqv5=0, mq9_5=0;
float dht_temp=0, dht_hum=0, bmp_temp=0, bmp_pre=0, rdy=0, ibme_temp=0, ibme_hum=0, tibme_hum=0, ilux=0, tilux=0;
float sbme_temp[S_MAX], sbme_hum[S_MAX], sbme_pre[S_MAX], silux[S_MAX], tibme_temp=0, ibme_pre=0, tibme_pre=0, h_cor=0;
volatile unsigned int loop_i = 0, i=0, loop_u = 0, s_i=0, ppress=0, menu_i = 0, menu_j = 0;


time_t tfs = 0, timecor=0, timea1pr=0;

volatile bool bmp_ok=false, lux_ok=false, dht_ok=false, data_rec=false, ibmp_ok=false, i_bool=false, ls_error = false;
volatile bool ispmode = false, drq = false, send_data = false, repsend = false, s_redy=false, no_opt=false, auto_led=false;
volatile bool loop_en=1, selfup=false, lcdbackl=true, data_get=true, narodmon_send=false, loop_u_new=0, narodmon_nts = true;
volatile bool ntp_error = false, but_reed = false, bpower = false, bup = false, bdn = false, menu_mode = false, offline = false;

char cstr1[BUF_SIZE], replyb[RBUF_SIZE], nreplyb[RBUF_SIZE], ctmp='\0';

char mac[22];
double rdtmp[3][RCOL+2];

void getd();
bool loadConfig();
bool saveConfig();
bool lcdbacklset();
bool lcdbacklset(bool);
unsigned int localPort = 2390;                              // local port to listen for UDP packets
IPAddress timeServerIP;

const int NTP_PACKET_SIZE = 48;                             // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE];                        //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

WiFiClient client;

void httpRequest();
unsigned long sendNTPpacket(IPAddress&);
int get_state(char *, unsigned int);
bool parse_A1DSP(char*);
bool parse_NAROD(char*);
bool A1_data_pr(char *, unsigned int);
bool NAROD_data_send(char *, short int);
void data_send_f();






long get_signal_qua(long rfrom, long rto){
    long rssi = WiFi.RSSI();
    if(rssi >= -40) 
    rssi = -40;
    else if(rssi <= -85) 
    rssi = -85;
    return map(rssi, -40, -85, rfrom, rto);
  }

static const PROGMEM char webPage[] ="<!DOCTYPE html>\n"
"<html>\n"
" <head>\n"
"  <meta charset=\"utf-8\">\n"
"  <title>ESPDISP Датчики</title>\n"
"  <link rel=\"shortcut icon\" href=\"favicon.ico\" />\n"
" </head>\n"
" <body>\n"
"  <h1>ESP8266 DISP</h1>\n"
"  <a href= \"/update\">Ручное обновление</a><br>\n"
"  <a href= \"/online_update\">Автоматическое обновление</a><br>\n"
"  <a href= \"/config.json\">Просмотр Json настроек</a><br>\n"
"  <a href= \"/a1pr\">A1_DSP</a><br>\n"
"  <a href= \"/replyb.txt\">Reply buffer</a><br>\n"
"  <a href= \"/sysinfo.txt\">Информация о модуле</a><br>\n"
"  <a href= \"/set?restart=1\">Перезагрузка</a><br>\n"
"  <a href= \"/set?backlight=0\">Отключить подсветку</a><br>\n"
"  <a href= \"/set?backlight=1\">Включить подсветку</a><br>\n"
"  <a href= \"/set?format=243\">Сброс настроек</a><br>\n"
" </body>\n"
"</html>\n";

const String authFailResponse1 = "<HTML>\n<HEAD><TITLE>401 Доступ запрещён</TITLE><meta charset=\"utf-8\"></HEAD>\n<BODY BGCOLOR=\"#cc9999\">\n<H4>401 Доступ запрещён</H4> Доступ к ";

const String authFailResponse2 = " запрещён.</BODY>\n</HTML>\n";

LiquidCrystal_I2C srlcd(0x20, 20, 2);

BH1750 lightMeter;
#define DNB 14
#define UPB 0
#define POWERB 2

void sendHeaders(){
        server.sendHeader("Server", "ESP8266/1.0.1", true);
		server.sendHeader("X-Content-Type-Options", "nosniff", false);
		server.sendHeader("Content-Language", "ru", false);
        server.sendHeader("Retry-After", "2", false);
		return;
}
void setup() {
    //Serial.begin();   
    //Serial.println("A1 DISP_ESP_ST");
    pinMode(POWERB, INPUT);
    pinMode(DNB, INPUT);
    pinMode(UPB, INPUT);
	digitalWrite(DNB, LOW);
	digitalWrite(UPB, LOW);
    yield();
    bzero(cstr1, BUF_SIZE);
    bzero(replyb, RBUF_SIZE);
    for(int r=0;r<RCOL;r++){
	  bzero(rdtmp[r],2);
    }
    srlcd.begin();
    srlcd.clear();
    srlcd.setCursor(0,0);
    srlcd.print(HOST_NAME);
    srlcd.setCursor(0,1);
    //Serial.println("Starting esp.");
    srlcd.print("Версия ");    
	sprintf(cstr1, "%d.%d.%d", fw_ver/100, (fw_ver%100)/10, fw_ver%10);
    srlcd.print(cstr1);
    delay(1000);
    bzero(cstr1, BUF_SIZE);
    srlcd.setCursor(0,1);
    srlcd.print("От ");
    srlcd.print(__TIME__"  ");
    delay(2000);
    srlcd.setCursor(0,1);
    srlcd.print("Запуск.              ");
    srlcd.setCursor(OFFSET,0);
    srlcd.print("WiFi");
    //Serial.println("WIFI");
    WiFi.begin();
    yield();
    srlcd.setCursor(7,1);
    srlcd.print(".");
    srlcd.backlight();
    srlcd.setCursor(OFFSET,0);
    srlcd.print("SPIFS");
    //Serial.println("SPIFS");
    
    if(ESP.getResetS() == false || digitalRead(POWERB) == LOW)
    {
        selfup=true;
        loop_en=false;
        srlcd.setCursor(0,1);
        srlcd.print("Ошибка посл выкл");
        delay(1000);
        srlcd.setCursor(0,1);
        srlcd.print("Принуд обн включено ");
        delay(1000);
    }
    if(selfup == false) {
	Wire.begin();
	ls_error = lightMeter.begin();
	bme.begin();  
	bme.setSampling(
	    Adafruit_BME280::MODE_FORCED,
	    Adafruit_BME280::SAMPLING_X1,
	    Adafruit_BME280::SAMPLING_X1,
	    Adafruit_BME280::SAMPLING_X1,
	    Adafruit_BME280::FILTER_OFF);
	bme.takeForcedMeasurement();
    if (!SPIFFS.begin()) {                                      //|| !digitalRead(BUT)) {
        //Serial.println("Failed to mount file system");
        SPIFFS.format();
        srlcd.setCursor(0,1);
        srlcd.print("Форматирование SPIFS");
      }

    srlcd.setCursor(OFFSET,0);
    srlcd.print("W СБРОС С1");
    yield();
    //Serial.println("W СБРС");
    srlcd.setCursor(8,1);
    srlcd.print(".");
    yield();

    WiFi.disconnect();
    yield();
    delay(200);
    yield();

    srlcd.setCursor(OFFSET,0);
    srlcd.print("ЗАГР КОНФ ");
    //Serial.println("CFG L");
    yield();
    srlcd.setCursor(9,1);
    srlcd.print(".");
	byte bmac[6];
	WiFi.macAddress(bmac);
	bzero(mac, 20);
	sprintf(mac, "%X-%X-%X-%X-%X-%X", bmac[0], bmac[1], bmac[2], bmac[3], bmac[4], bmac[5]);
    if (!loadConfig()) {
        srlcd.setCursor(0,1);
        srlcd.print("ОШИБКА, СБРОС");
        SPIFFS.remove("/config.json");
        if (!saveConfig()) {
    	    srlcd.setCursor(0,1);
    	    srlcd.print("НЕ ВОЗМОЖНО СОХР КОНФ");
            //Serial.println("Failed to save config");
            SPIFFS.format();
			} 
		else {
    	      srlcd.setCursor(0,1);
    	      srlcd.print("УСП СОХР КОНФ");
              //Serial.println("Config saved");
            }
        //Serial.println("Failed to load config");
      }
    srlcd.setCursor(OFFSET,0);
    //Serial.println("W STA ");
    srlcd.print("W СБРОС С2");
    srlcd.setCursor(10,1);
    srlcd.print(".");
    }
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false);
    srlcd.setCursor(11,1);
    srlcd.print(".");
    WiFi.setAutoReconnect (true);
    delay(100);
    srlcd.setCursor(12,1);
    srlcd.print(".");
    srlcd.setCursor(OFFSET,0);
    srlcd.print("WAN КЛИЕНТ");
    srlcd.setCursor(0,1);
    srlcd.print("ПОПЫТКА ПОДКЛЮЧЕНИЯ ");
    WiFi.hostname(HOST_NAME);
    WiFi.begin("A1 Net", "84992434219");
    int t = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        //Serial.print(".");
        t++;
        if (t == 20) {
            srlcd.setCursor(OFFSET,0);
            srlcd.print("WAN ТД    ");
            srlcd.setCursor(0,1);
            srlcd.print("ПРЕРХОД В РЕЖИМ ТД  ");
            //loop_en=false;
			offline = true;
            WiFi.softAP(HOST_NAME, sta_password);
            IPAddress myIP = WiFi.softAPIP();
			break;
          }
      }
    //Serial.println("WiFi connected");
    //Serial.println("IP address: ");
    //Serial.println(WiFi.localIP());

    srlcd.setCursor(OFFSET,0);
    srlcd.print("ЗАПУСК WEB");
    srlcd.setCursor(0,1);
    srlcd.print("Запуск сервера  ");
	
    server.on("/", []() {
		sendHeaders();
		//server.sendContent_P(200, "image/x-icon", (const char*)favicon_ico, favicon_ico_len);
        server.send_P(200, "text/html; charset=utf-8", webPage);
        //Serial.printf("Stations connected = %d\n", WiFi.softAPgetStationNum());
      });

    server.on("/config.json", []() {
		sendHeaders();
        File file = SPIFFS.open("/config.json", "r");
        size_t sent = server.streamFile(file, "application/json; charset=utf-8");
        file.close();
        delay(1000);
      });
	  
	  
	  

server.on("/i2c", []() {
	 sendHeaders();
	 bzero(cstr1, BUF_SIZE);
     sprintf(cstr1, "-");
     
	  uint8_t portArray[] = {5, 4};
		
		
  for (uint8_t i = 0; i < sizeof(portArray); i++) {
    for (uint8_t j = 0; j < sizeof(portArray); j++) {
      if (i != j){
        sprintf(cstr1, "%s\nScanning (SDA : SCL) - %d : %d - ", cstr1, portArray[i],portArray[j]);
        Wire.begin(portArray[i], portArray[j]);
          byte error, address;
  int nDevices;
  nDevices = 0;
  for (address = 1; address < 127; address++ )  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0){
      sprintf(cstr1, "%s\nI2C device found at address 0x", cstr1);
      if (address < 16)
        sprintf(cstr1, "%s\n0", cstr1);
      sprintf(cstr1, "%s%x\n",cstr1 ,address);

      nDevices++;
    } else if (error == 4) {
      sprintf(cstr1, "%s\nUnknow error at address 0x", cstr1);
      if (address < 16)
        sprintf(cstr1, "%s\n0", cstr1);
      sprintf(cstr1, "%s%x\n",cstr1 ,address);
    }
  } //for loop
  if (nDevices == 0)
    sprintf(cstr1,"%sNo I2C devices found\n", cstr1);
  else
    sprintf(cstr1,"%s**********************************\n", cstr1);
      }
    }
  }
	server.send(200, "text/plain; charset=utf-8", cstr1);
	delay(1000);
});

server.on("/xml.xml", []() {  //Инициализация обработчика страници XML с данными датчиков
  get_state(cstr1, BUF_SIZE); //Формированиме XML
  server.send(200, "text/xml", cstr1); //Отправка данных клиенту
  delay(1000);
});
  
server.on("/favicon.ico", []() {
	sendHeaders();
	server.send_P(200, "image/x-icon", (const char*)favicon_ico, favicon_ico_len);
	delay(1000);
});   

server.on("/sysinfo.txt", []() {
		sendHeaders();
        server.sendHeader("Refresh", "10", false);
        bzero(cstr1, BUF_SIZE);
        rtc.refresh();
        tfs = millis()/1000;
		sprintf(cstr1, "Hostname: %s\nFW Ver: %d.%d.%d\n"
		 "Comp info, time: " __TIME__ ", date: " __DATE__ "\n"
         "Time cor a1pr: %u\n"
		 "MAC: %s\n"
		 "SketchSize: %d\n"
         "SketchMD5: %s\n"
         "CPU Frq: %d MHz\n"
         "Free Heap: %d\n"
         "ESP VCC: %.2f\n"
		 "Light sensor error: %d\n"
		 "Int light sens: %dlux\n"
         "Int BMP OK: %d\n"
         "Int BMP Pressure: %.2fmmHg\n"
         "Int BMP Hum: %.2f%c\n"
         "Int BMP Temp: %.2f°C\n"
         "Int BMP S REDY: %d\n"
         "Int BMP Hum: %.2f%c\n"
         "Int BMP S Temp: %.2f°C\n"
         "Time from start: %lu hours %lu minutes %lu Sec, and %lu days \n"
         "ESP Chip ID: %X\n"
         "Flash id: %X\n"
         "Flash real size: %d\n"
         "Flash ide size: %d\n"
         "Flash ide speed: %d\n"
         "Flash ide mode: %s\n"
         "Last reset: %s(%d)\n"
         "Reset info: %s\n"
         "WiFi Status: %d\n"
         "SSID: %s\n"
         "RSSI: %d dbm\n"
         "data get: %d\n"
		 "Repet send: %d\n"
		 "Loop enable: %d\n"
         "Signal quality: %d %%\n"
		 "RTC Time: %d hours %d minutes %d second %d year %d month %d day\n"
         "Last Reply: %s\n", HOST_NAME, fw_ver/100, (fw_ver%100)/10, fw_ver%10, timea1pr,
		 mac, ESP.getSketchSize(), ESP.getSketchMD5().c_str(), ESP.getCpuFreqMHz(),
		 ESP.getFreeHeap(), esp_vcc, ls_error, lightMeter.readLightLevel(), ibmp_ok, ibme_pre, ibme_hum, 0x25, ibme_temp,  s_redy, tibme_hum,
		 0x25, tibme_temp, numberOfHours(tfs), numberOfMinutes(tfs), numberOfSeconds(tfs), elapsedDays(tfs),
		 ESP.getChipId(), ESP.getFlashChipId(), ESP.getFlashChipRealSize(), ESP.getFlashChipSize(), ESP.getFlashChipSpeed(),
		 (ESP.getFlashChipMode() == FM_QIO ? "QIO" : ESP.getFlashChipMode() == FM_QOUT ? "QOUT" : ESP.getFlashChipMode() == FM_DIO ? "DIO" : ESP.getFlashChipMode() == FM_DOUT ? "DOUT" : "UNKNOWN"),
		 ESP.getResetReason().c_str(), ESP.getResetS(), ESP.getResetInfo().c_str(), WiFi.status(),
		 WiFi.SSID().c_str(), WiFi.RSSI(), data_get, repsend, loop_en, get_signal_qua(100, 0), rtc.hour(), 
		 rtc.minute(), rtc.second(), rtc.year(), rtc.month(), rtc.day(), replyb);
        server.send(200, "text/plain; charset=utf-8", cstr1);
        delay(1000);
      });
    server.on("/nreplyb.txt", []() {
		sendHeaders();
        server.send(200, "text/plain; charset=utf-8", nreplyb);
        delay(1000);
      });
    server.on("/replyb.txt", []() {
		sendHeaders();
        server.send(200, "text/plain; charset=utf-8", replyb);
        delay(1000);
      });
	  
    server.on("/a1pr", []() {
		sendHeaders();
		A1_data_pr(cstr1, BUF_SIZE);
		server.send(200, "text/plain", cstr1);
		delay(1000);
      });
	
    server.on("/online_update", []() {
		if (!server.authenticate(www_username, www_password)) {
			return server.requestAuthentication(DIGEST_AUTH, "Требуется авторизация", authFailResponse1+server.uri()+authFailResponse2);
		}
		sendHeaders();
        loop_en=false;
        server.send(200, "text/plain; charset=utf-8", "Try to selfupdate...\n");
        delay(1000);
        selfup=true;
      });

    server.on("/set", []() {
		if (!server.authenticate(www_username, www_password)) {
			return server.requestAuthentication(DIGEST_AUTH, "Требуется авторизация", authFailResponse1+server.uri()+authFailResponse2);
		}
		sendHeaders();
        if (server.arg("restart") != "") {
            server.send(200, "text/plain; charset=utf-8", "RESTARTING...\n");
            delay(1000);
            ESP.restart();
          }
        if (server.arg("format") != "") {
            if(atoi(server.arg("pass").c_str()) == 243) {
                server.send(200, "text/plain; charset=utf-8", "Starting format...\n");
                delay(1000);
                SPIFFS.format();
                ESP.restart();
              }
          }
        if (server.arg("v_mode") != "") {
            WiFi.disconnect(false);
            WiFi.begin(wifi_name.c_str(), wifi_password.c_str());
          }
		  
        if (server.arg("net_name") != "") {
        if (server.arg("pass") != "") {
              wifi_name = server.arg("net_name");
              wifi_password = server.arg("pass");
              saveConfig();
              WiFi.disconnect(false);
              WiFi.begin(wifi_name.c_str(), wifi_password.c_str());
            }
          }
        if (server.arg("backlight") != "") {
            lcdbacklset(tobool(server.arg("backlight").c_str()));
        }
        if (server.arg("DEBUG") != "") {
            DEBUG=tobool(server.arg("DEBUG").c_str());
        }
		
		if (server.arg("loop_en") != "") {
            loop_en = tobool(server.arg("loop_en").c_str());
        }
		
		if (server.arg("narodmon_nts") != "") {
            narodmon_nts = tobool(server.arg("narodmon_nts").c_str());
        }
		
		if (server.arg("auto_led") != "") {
            auto_led = tobool(server.arg("auto_led").c_str());
        }
		
        server.send_P(200, "text/html", webPage);
        delay(500);
        saveConfig();
      });

    server.begin();
    httpUpdater.setup(&server, www_username, www_password);
	
	srlcd.setCursor(OFFSET,0);
    srlcd.print("ПОДКЛЮЧ ПРИРЫВ");
	attachInterrupt(digitalPinToInterrupt(DNB), DNBhandleInt, FALLING);
	attachInterrupt(digitalPinToInterrupt(UPB), UPBhandleInt, FALLING);
	attachInterrupt(digitalPinToInterrupt(POWERB), POWERBhandleInt, FALLING);
	
    srlcd.setCursor(OFFSET,0);
    srlcd.print("НОРМ РЕЖИМ");
    srlcd.setCursor(0,1);
    srlcd.print("ПРЕХОД В НОРМ РЕЖИМ ");
    udp.begin(localPort);
    srlcd.setCursor(0,1);
    srlcd.print("Запуск ntp          ");

    WiFi.hostByName(ntpServerName, timeServerIP); 
	sendNTPpacket(timeServerIP);                                // send an NTP packet to a time server
    delay(1000);
    int cb = udp.parsePacket();
    if (cb) {
        //Serial.println("no packet yet");
        udp.read(packetBuffer, NTP_PACKET_SIZE);
		
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        timecor = highWord << 16 | lowWord;
	    setTime(timecor - 2208988800UL + timeZone * SECS_PER_HOUR);
	    rtc.set(second(), minute(), hour(), weekday(), day(), month(), year());
        timecor = timecor - (millis()/1000);
		//  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
        ntp_error = false;
      }
	else
	{
		srlcd.setCursor(0,1);
		srlcd.print("Ошибка ntp          ");
		ntp_error = true;
		rtc.refresh();
		if(rtc.year() > 2000){
			srlcd.setCursor(0,1);
			srlcd.print("Установка по rtc    ");
			rtc.refresh();
			setTime(rtc.hour(), rtc.minute(), rtc.second(), rtc.day(), rtc.month(), rtc.year());
			timecor = (now() + 2208988800UL) - (millis()/1000);
			
		}
	}
	
	
    udp.stop();
    srlcd.clear();
	data_collect.attach(5, getd);
	data_send_tic.attach(300, data_send_f);
  }

void loop() {
    yield();
    server.handleClient();
	if(WiFi.status() != WL_CONNECTED) {
		offline = true; }
	else {
		offline = false; }
	
	if(narodmon_send==true) {
	  repsend = !NAROD_data_send(cstr1, BUF_SIZE);
	  narodmon_send=false;
	}
    if(loop_en == true) {
		if(data_get==true) {
			esp_vcc = ESP.getVcc()/1000.0;
			ilux = lightMeter.readLightLevel();
			
			if(auto_led == true)
			{
				if(ilux > 40) {
					lcdbacklset(false);
				}
				else {
					lcdbacklset(true);
				}
			}
		
		
			bme.takeForcedMeasurement();
			tmp=bme.readHumidity();
			if(tmp < 0 || tmp > 100 || tmp == NAN){
				ibmp_ok=false;
			}
			if(tmp > 0 && tmp < 100 && tmp != NAN){
				ibme_temp = bme.readTemperature()-2;
				ibme_pre = bme.readPressure()/133.322;
				ibme_hum = tmp;
				ibmp_ok=true;
			}
			if(ibme_temp == 0 && ibme_hum == 0){
				ibmp_ok=false;
			}
			data_get=false;
			if(ibmp_ok==1){
			silux[s_i]=ilux;
			sbme_hum[s_i]=ibme_hum;
			sbme_temp[s_i]=ibme_temp;
			sbme_pre[s_i]=ibme_pre;
			if(s_i >= S_MAX) {
				s_redy=1;
				s_i=0;
			}
			else {
				s_i++;
			}
		}
		if(s_redy==1) {
				//tilux=ilux;
				tilux=get_scsf(silux,S_MAX);
				tibme_hum=get_scsf(sbme_hum,S_MAX);
				tibme_temp=get_scsf(sbme_temp,S_MAX);
				tibme_pre=get_scsf(sbme_pre,S_MAX);
			}
		else {
				tilux=ilux;
				tibme_hum=ibme_hum;
				tibme_temp=ibme_temp;
				tibme_pre=ibme_pre;
			}
		}
        if(client.available() && offline == false) {
            bzero(replyb, RBUF_SIZE);
            for(i = 0;client.available() && i < RBUF_SIZE;i++)
            {
                replyb[i] = client.read();
            }
            if(i <= (RBUF_SIZE-2))
                replyb[i+1]='\0';
            else
                replyb[RBUF_SIZE-1]='\0';
            if(replyb[13]== 'O' && replyb[14]== 'K')
            {
				int y =0;
				for(i=0;i<(strlen(replyb)-4); i++)
				{
					if(replyb[i]== ';' && replyb[i+1]== 0x0a ) {
						break; }
					if(replyb[i]== 0x0d && replyb[i+1]== 0x0a && replyb[i+2]== 0x0d && replyb[i+3]== 0x0a)
					{
						y=i+4;
					}
				}
                for(i=y; i < strlen(replyb); i++)
                {
                    replyb[i-y] = replyb[i];
                    if(replyb[i] == ';')
                    {
                        if(i<(RBUF_SIZE-1))
                        {
                	    i++;
                            for(i = i-y; i < RBUF_SIZE; i++){
                               replyb[i]='\0';}
                            parse_A1DSP(replyb);
                        }
                        break;
                    }
                }
            }
          }
    }
	
	delay(500);
    if(loop_i > 20){
		if(offline == false) {
			httpRequest(); }
        loop_i = 0;
        loop_u++;
	    loop_u_new=1;
      }
    yield();
    
    if(selfup==true && offline == false) {
        ESP.wdtDisable();
        srlcd.clear();
        srlcd.setCursor(0,0);
        ESPhttpUpdate.rebootOnUpdate(false);
        srlcd.print("Запуск обновления... ");
        delay(2000);
        t_httpUpdate_return ret = ESPhttpUpdate.update("http://dev.a1mc.ru/rom/esp8266/disp/flash.bin");
        srlcd.setCursor(0,1);
        switch(ret) {
            case HTTP_UPDATE_FAILED:
                srlcd.print("Ошибка обновления:  ");
                delay(2000);
                srlcd.print(ESPhttpUpdate.getLastErrorString().c_str());
                break;

            case HTTP_UPDATE_NO_UPDATES:
                srlcd.print("Обн. не требуется   ");
                break;

            case HTTP_UPDATE_OK:
                srlcd.print("Обновлено успешно   ");
                delay(2000);
                srlcd.setCursor(0,1);
                srlcd.print("Перезагружаюсь...   ");
                delay(2000);
                ESP.restart();
                break;
        }
        selfup=false;
        delay(2000);
        srlcd.clear();
        loop_en = true;
    }
    if(loop_en == true) {
        //if(dht_ok == 1){
		if(loop_u_new==1){
			loop_u_new=0;
            char sm[2] = {' ', ' '};
            srlcd.setCursor(0,1);
            yield();
            if(loop_u==1){
				if(offline == true){
					loop_u = 4;}
                sprintf(cstr1, "Влажн: %.2f", dht_hum);
                sm[0]=0x25;
              }
            if(loop_u==2){
                sprintf(cstr1, "Осв: %.2fлкс", lux);} 
            else if(loop_u==3){
                sprintf(cstr1, "Темп: %.2f", dht_temp);
                sm[0]=0x99;
                sm[1]='C';
              }
			else if(loop_u==4 && ibmp_ok == true){
				sprintf(cstr1, "Вн темп: %.2f", tibme_temp);
				sm[0]=0x99;
				sm[1]='C';
			}
			else if(loop_u==5 && ibmp_ok == true){
				sprintf(cstr1, "Вн влажн: %.2f", tibme_hum);
				sm[0]=0x25;
			}
			
            else {
                sprintf(cstr1, "Давлен: %.2f%ммРтСт", tibme_pre);
                loop_u=1;
              }
			if(loop_u_new==0) {
				ppress=0;
				srlcd.print(cstr1);
				srlcd.writecode(sm[0]);
				srlcd.writecode(sm[1]);
				for( i=strlen(cstr1); i <= 22; i++)
				{
					srlcd.write(' ');
				}
			}
			loop_u_new=0;
          }
        yield();
        unsigned long secsSince1900 = timecor + (millis()/1000);
        
        srlcd.setCursor(18,0);
		if(offline != true) {
			switch(get_signal_qua(6, 0))
			{
            case 0:
            srlcd.writecode(0x80);
            srlcd.writecode(0x80);
                break;
            case 1:
            srlcd.writecode(0x81);
            srlcd.writecode(0x80);
                break;
            case 2:
            srlcd.writecode(0x82);
            srlcd.writecode(0x80);
                break;
            case 3:
            srlcd.writecode(0x83);
            srlcd.writecode(0x80);
                break;
            case 4:
            srlcd.writecode(0x83);
            srlcd.writecode(0x81);
                break;
            case 5:
            srlcd.writecode(0x83);
            srlcd.writecode(0x82);
                break;
            case 6:
            srlcd.writecode(0x83);
            srlcd.writecode(0x83);
                break;
			}
		}
		else {
            srlcd.writecode(0x20);
            srlcd.writecode(0xfc);
		}
        // now convert NTP time into everyday time:
        // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
        const unsigned long seventyYears = 2208988800UL;
        // subtract seventy years:
        unsigned long epoch = secsSince1900 - seventyYears;
        //srlcd.setCursor(0,0);
	//	srlcd.print(dht.computeHeatIndex(ibme_temp, ibme_hum, false));
        srlcd.setCursor(5,0);
		srlcd.print(" ");
        i=numberOfHours(epoch);
        i+=timeZone;
        if(i>23) {
            i-=24;
          }
        if ( i < 10 ) {
            // In the first 10 minutes of each hour, we'll want a leading '0'
            srlcd.print('0');
          }
        srlcd.print(i);
        srlcd.print(':');
        if ( numberOfMinutes(epoch) < 10 ) {
            // In the first 10 minutes of each hour, we'll want a leading '0'
            srlcd.print('0');
          }
        srlcd.print(numberOfMinutes(epoch));
        srlcd.print(':');
        if ( numberOfSeconds(epoch) < 10 ) {
            // In the first 10 seconds of each minute, we'll want a leading '0'
            srlcd.print('0');
          }
        srlcd.print(numberOfSeconds(epoch));
      }
	if(menu_mode == true){
		bzero(cstr1, BUF_SIZE);
		
		srlcd.setCursor(0, 0);
		sprintf(cstr1, "Настройки");
		srlcd.print(cstr1);
		for(i=(20-strlen(cstr1)); i !=0; i--)
		{
			srlcd.write(' ');
		}
		loop_en = false;
		
		srlcd.setCursor(0, 1);
		
		if(loop_i == 19)
		{
			srlcd.clear();
			bzero(cstr1, BUF_SIZE);
			menu_mode = false;
            loop_en = true;			
		}
		no_opt=false;
		if(loop_u_new == 1) {
			loop_u_new = 0;
			if(loop_u==1){
                sprintf(cstr1, "Отл режим: ");
				if(bpower == true) {
				    ppress = 0;
					bpower = false;
					DEBUG = !DEBUG;
				}
				i_bool = DEBUG;
            }
            else if(loop_u==2){
                sprintf(cstr1, "Подсветка: ");	
				ppress = 0;			
				if(bpower == true) {
					bpower = false;
					lcdbacklset(!lcdbacklset());
				}
				i_bool = lcdbacklset();
		    }
            else if(loop_u==3){
                sprintf(cstr1, "Отп narodmon: ");	
				ppress = 0;	
				if(bpower == true) {
					bpower = false;
					narodmon_nts = !narodmon_nts;
				}
				i_bool = narodmon_nts;
            }
			else if(loop_u==4){
                sprintf(cstr1, "Обновление: ");		
				ppress = 0;
				if(bpower == true) {
					bpower = false;
					selfup = !selfup;
				}
				no_opt=true;
				i_bool = selfup;
			}
			else if(loop_u==5){
				float h = 8000*((1+tibme_temp*(1/273))/(bmp_pre)); //-tibme_pre
				h = h - h_cor;
				sprintf(cstr1, "Высота: %f", h);
				ppress = 0;	
				if(bpower == true) {
					bpower = false;
					h_cor = h;
				}
				no_opt=true;
				saveConfig();
			}
            else {
				sprintf(cstr1, "Выход");		
				ppress = 0;
				if(bpower == true) {
					bpower = false;
					srlcd.clear();
					bzero(cstr1, BUF_SIZE);
					menu_mode = false;
					loop_en = true;	
					delay(1000);
				}
				no_opt=true;
                loop_u=0;
            }
			 
			if(loop_u_new==0 && menu_mode == true) {
				srlcd.writecode(0x84);
				srlcd.print(cstr1);
				if(no_opt==false)
					print_bool(i_bool);
				saveConfig();
				for( i=strlen(cstr1); i <= 24; i++)
				{
					srlcd.write(' ');
				}
				srlcd.setCursor(19, 1);
				srlcd.writecode(0x85);
			}
			  
		}
	}
	yield();
	if(but_reed == true && DEBUG == true) {
			srlcd.setCursor(0,0);
			sprintf(cstr1, "%d%d%d", digitalRead(POWERB), digitalRead(UPB), digitalRead(DNB));
			srlcd.print(cstr1);
			but_reed = false;
	}
    loop_i++;
  }
  
void print_bool(bool stat) {
	if(stat == true)
		srlcd.print("Вкл ");
	else
		srlcd.print("Выкл");
	return;
}

bool parse_A1DSP(char* tempstr) {
    //rx входная строка, rs колличество символов в строке, rc количество параметров
	
    /*srlcd.setCursor(OFFSET,0);
    srlcd.print("ПАРС А1ПРО");
	delay(1000);*/
    bmp_ok=false;
    lux_ok=false;
    dht_ok=false;
	int st_col = 0;
	for(int i =0; i<strlen(tempstr); i++)
	{
		if(tempstr[i] == ':') {
			st_col++; }
	}
    if(st_col<1 || st_col > 25)
		return false;
	
	yield();
	int col=st_col;
	st_col++;
    int i = 0;
    float *dat_mas = (float *)malloc(st_col * sizeof(float));
/*    for(i=0 ; i < st_col; i ++)
    {
	dat_mas[i]=0;
    }*/
	char **name_mas = (char **)malloc(st_col * sizeof(char *));
	for(i = 0; i < st_col; i++) {
		name_mas[i] = (char *)malloc(15 * sizeof(char));
	}
	
    /*srlcd.setCursor(OFFSET,0);
    srlcd.print("РАЗД А1ПРО");
	delay(1000);*/
	splint_rtoa(tempstr, strlen(tempstr), col, name_mas, dat_mas);
	//return col;
	yield();
	if(col > 0) {
		data_rec=true;
		for(int ilp = 0; ilp < col; ilp++) {
			yield();
			//tempstr += String("\nName = ") + name_mas[ilp] + String(" data = ") + dat_mas[ilp];
			if (strcmp(name_mas[ilp], "RDY") == 0) {
				rdy=dat_mas[ilp];
			}
			else if (strcmp(name_mas[ilp], "MQV5") == 0)  {
				mqv5=dat_mas[ilp];
			}
			else if (strcmp(name_mas[ilp], "MQV") == 0)  {
				mqv=dat_mas[ilp];
			}
			else if (strcmp(name_mas[ilp], "VMQ7") == 0) {
				mq7=dat_mas[ilp];
			}
			else if (strcmp(name_mas[ilp], "VMQ9") == 0) {
				mq9=dat_mas[ilp];
			}
			else if (strcmp(name_mas[ilp], "VMQ9_5") == 0) {
				mq9_5=dat_mas[ilp];
			}
			else if (strcmp(name_mas[ilp], "VIN") == 0)  {
				vin=dat_mas[ilp];
			}
			else if (strcmp(name_mas[ilp], "MCVCC") == 0){
				mc_vcc=dat_mas[ilp];
			}
			else if (strcmp(name_mas[ilp], "MCTMP") == 0){
				mc_temp=dat_mas[ilp];
			}
			else if (strcmp(name_mas[ilp], "LUX") == 0)  {
				lux=dat_mas[ilp];
				lux_ok=true;
			}
			else if (strcmp(name_mas[ilp], "HUM") == 0) {
				dht_hum=dat_mas[ilp];
				dht_ok=true;
			}
			else if (strcmp(name_mas[ilp], "TEMP") == 0) {
				dht_temp=dat_mas[ilp];
				dht_ok=true;
			}
			else if (strcmp(name_mas[ilp], "BPRE") == 0) {
				bmp_pre=dat_mas[ilp];
				bmp_ok=true;
			}
			else if (strcmp(name_mas[ilp], "RFWVER") == 0) {
				if(dat_mas[ilp] > fw_ver){
					srlcd.setCursor(0,1);
					srlcd.print(dat_mas[ilp]);
					srlcd.print(">");
					srlcd.print(fw_ver);
					delay(2000);
					selfup=true;}
			}
			else if (strcmp(name_mas[ilp], "BTMP") == 0) {
				bmp_temp=dat_mas[ilp];
				bmp_ok=true;
			}
            else if(strcmp(name_mas[ilp], "time") == 0) {
                timea1pr=dat_mas[ilp]*100000;
                if(ntp_error == true){
                    timecor=timea1pr+2208988800ul;
                    ntp_error = false;
                    srlcd.setCursor(0,1);
                    srlcd.print("Уст врем через a1pr ");
					
					setTime(timea1pr + timeZone * SECS_PER_HOUR);
					rtc.set(second(), minute(), hour(), weekday(), day(), month(), year());
					
                    timecor = timecor - (millis()/1000);
                }
			}
		}	
	}
	else {
		data_rec=false;
	}
	
	free(dat_mas);
	for(i=0; i < st_col; i++) {
		free(name_mas[i]);
	}
	free(name_mas);
	
	return data_rec;
	
}

unsigned long sendNTPpacket(IPAddress& address){
    //Serial.println("[NTP]sending NTP packet...");
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;                               // LI, Version, Mode
    packetBuffer[1] = 0;                                        // Stratum, or type of clock
    packetBuffer[2] = 6;                                        // Polling Interval
    packetBuffer[3] = 0xEC;                                     // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123);                              //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
}

void httpRequest() {
    // close any connection before send a new request.
    // This will free the socket on the WiFi shield
    client.stop();

    // if there's a successful connection:
    if (client.connect("dev.a1mc.ru", 80)) {
        ////Serial.println("connecting...");
        // send the HTTP GET request:
        client.println("GET /kd2.php HTTP/1.1");
        client.println("Host: dev.a1mc.ru");
        client.print("User-Agent: ");
        client.print(HOST_NAME);
        client.println("/1.1");
        client.println("Accept: text/plain, text/html");
        client.println("Connection: close");
        client.println();
      } else {
          // if you couldn't make a connection:
          ////Serial.println("connection failed");
        }
  }


bool loadConfig() {
    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile) {
        return false;
      }

    size_t size = configFile.size();
    if (size > 512) {
		configFile.close();
        return false;
      }


    StaticJsonBuffer<512> jsonBuffer;
    JsonObject &json = jsonBuffer.parseObject(configFile);
	
	

    if (!json.success()) 
        return false;
	
    if(json["auto_led"]==NULL)
	return false;
    if(json["fw_ver"]==NULL) 
	return false;
    if(atoi(json["fw_ver"]) < fw_ver)
	//return false;
    auto_led = tobool(json["auto_led"]);
    lcdbacklset(tobool(json["lcdbackl"]));
    DEBUG=tobool(json["DEBUG"]);
	narodmon_nts = tobool(json["narodmon_nts"]);

	 
    return true;
  }

bool saveConfig() {
    StaticJsonBuffer<512> jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
        return false;
    }
    json["fw_ver"] = fw_ver;
	json["auto_led"] = auto_led;
    json["lcdbackl"] = lcdbacklset();
    json["wifi_name"] = wifi_name;
    json["wifi_password"] = wifi_password;
	json["DEBUG"] = DEBUG;
	json["narodmon_nts"] = narodmon_nts;
    json.printTo(configFile);
	configFile.close();
    return true;
  }
  
void getd()	{
	data_get=true;
	return;
}

bool A1_data_pr(char *s, unsigned int s_size) {
  bzero(s, s_size);
  sprintf(s,
		  "EVC:%f RSSI:%d", esp_vcc, WiFi.RSSI());
	if(ibmp_ok == true) {
	sprintf(s,
			"%s TMP:%f"
			  " HUM:%f"
			  " PRE:%f", s, tibme_temp, tibme_hum, ibme_pre);
	}
  sprintf(s, "%s LUX:%f", s, tilux);
  sprintf(s, "%s FW:%d", s, fw_ver);
  sprintf(s, "%s ;\n\0", s);
  return 0;
}

bool NAROD_data_send(char *str,short int size) {
  WiFiClient client;
  bzero(str, size);
  
  byte bmac[6];
  WiFi.macAddress(bmac);
  bzero(mac, 20);
  sprintf(mac, "%X-%X-%X-%X-%X-%X", bmac[0], bmac[1], bmac[2], bmac[3], bmac[4], bmac[5]);
  sprintf(str,
          "#%X-%X-%X-%X-%X-%X#%s_v%d.%d.%d\n"
		  "#WSQ#%d#Качество сигнала WAN\n"
		  "#FHED#%d#Free ram in byte\n"
		  "#lcdbackl#%d\n", bmac[0], bmac[1], bmac[2], bmac[3], bmac[4], bmac[5], HOST_NAME, fw_ver/100, (fw_ver%100)/10, fw_ver%10, WiFi.RSSI(), ESP.getFreeHeap(),lcdbacklset());
  if(ibmp_ok == true) {
	sprintf(str,
			"%s#DTMP#%.2f#Температура\n"
			  "#DHUM#%.2f#Влажность\n"
			  "#DPRE#%.2f#Давление\n"
			  "#DLUX#%.2f#Освещённость\n"
			  , str, tibme_temp, tibme_hum, ibme_pre, tilux);
  }
  sprintf(str, "%s##\n\0", str);
  
  client.connect("narodmon.ru", 8283);											
  client.print(str);  
  
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 1000) {
      client.stop();
	  return false;
    }
  }
  
  for(i = 0; i < (RBUF_SIZE - 10); i++)
  {
	  nreplyb[i]=client.read();
	  if(nreplyb[i] == '\r' || nreplyb[i] == '\0' || nreplyb[i] == '\n')
		  break;
  }
  nreplyb[i]='\0';
  if(nreplyb[0] == 'O' && nreplyb[1] == 'K') {
	return true;
  }
  else if(nreplyb[0] == '#') {
	  parse_NAROD(nreplyb);
		return true;  
  }
  
  
  char tmp = nreplyb[7];
  nreplyb[7]='\0';
  if(i > 8 && strcmp(nreplyb,"INTERVAL")){
    nreplyb[7]=tmp;
	return true;  
  }
  nreplyb[7]=tmp;
  if(nreplyb[0]== '#') {
	  parse_NAROD(nreplyb);
		return true;  
  }
  
  return false;
}

void data_send_f() {
	if(narodmon_nts == true) {
		narodmon_send = true;
	}
	return;
}

bool parse_NAROD(char* tempstr) {
    bool data_rec=false;
 	int st_col = 0;
 	
 	for(int i =0; i<strlen(tempstr); i++)
	{
		if(tempstr[i] == '=') {
			st_col++; }
	}
    if(st_col<0 || st_col > 25)
		return false;
	
	yield();
	int col=st_col;
    int i = 0;
    
    int *dat_mas = (int *)malloc(st_col * sizeof(int));
	char **name_mas = (char **)malloc(st_col * sizeof(char *));
	for(i = 0; i < st_col; i++) {
		name_mas[i] = (char *)malloc(15 * sizeof(char));
	}
	
	for(int t = 0; t < (strlen(tempstr)-1); t++){
		tempstr[t]=tempstr[t+1];
	}
	tempstr[strlen(tempstr)-1]='\0';
	splint_narod(tempstr, strlen(tempstr), col, name_mas, dat_mas);
	yield();
	if(col > 0) {
		data_rec=true;
		for(int ilp = 0; ilp < col; ilp++) {
			yield();
			if (strcmp(name_mas[ilp], "lcdbackl") == 0) {
				lcdbacklset(dat_mas[ilp]);
				saveConfig();
			}		
		}	
	}
	else {
		data_rec=false;
	}
	free(dat_mas);
	for(i=0; i < st_col; i++) {
		free(name_mas[i]);
	}
	free(name_mas);

	return data_rec;
}

bool lcdbacklset(bool bkl){
 switch (bkl) {
	case 1:
		lcdbackl=true;
		srlcd.backlight();
		break;
       case 0:
		srlcd.noBacklight();
		lcdbackl=false;
		break;
		}
 return lcdbackl;
}

bool lcdbacklset(){
	return lcdbackl;
}



void POWERBhandleInt()
{
  bpower = true;
  but_reed=true;
  buthandleInterrupts();	
  ppress ++;
  loop_i = 0;
//  if(ppress == 1)
//	  loop_u_new=1;
  if(ppress == 1) 
  {
	  loop_u_new=1;
	  if(menu_mode != true) {
	  bpower = 0;
	  srlcd.clear();
	  menu_mode = true;}
  }
  if(ppress > 3)
  {
	  ESP.restart();
  }
  return;
}
void UPBhandleInt()
{   
  loop_i = 0;
  if(loop_u < 5)
		loop_u++;
	else
		loop_u = 0;
	loop_u_new=1;
  bup = true;
  but_reed=true;
  buthandleInterrupts();	
  return;
}
void DNBhandleInt()
{ 
  loop_i = 0;
  if(loop_u > 0)
		loop_u--;
	else
		loop_u = 5;
	loop_u_new=1;
  bdn = true;
  but_reed=true;	
  buthandleInterrupts();
  return;
}

void buthandleInterrupts()
{
  if(DEBUG == true) {
  but_reed=true; }
  return;
}

int get_state(char *s, unsigned int s_size) { //Формирование XML на основе шаблона
  sprintf(s,
          "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
		  "<esp>\n"
		  " <fw>\n"
		  "  <b_time>" __TIME__ "</b_time>\n"
		  "  <b_date>" __DATE__ "</b_date>\n"
		  "  <ver>%d</ver>\n"
		  " </fw>\n"
		  " <status>\n"
		  "  <esp_vcc>%f</esp_vcc>\n"
		  "  <esp_rssi>%d</esp_rssi>\n", fw_ver, esp_vcc, WiFi.RSSI());
		  
  bool ilux_ok = 1;
  if(ilux == 65535)
	  ilux_ok =0;
  sprintf(s, "%s  <bmp_r>%d</bmp_r>\n", s, ibmp_ok);
  sprintf(s, "%s  <bh1750_r>%d</bh1750_r>\n", s, ilux_ok);
  sprintf(s, "%s </status>\n", s);  
  sprintf(s, "%s <sensors>\n", s);
  
  if(ibmp_ok == true) {
	sprintf(s, "%s  <bmp>\n",s);
	sprintf(s,
		  "%s   <temp>%f</temp>\n"
			"   <pre>%f</pre>\n"
			"   <hum>%f</hum>\n", s, tibme_temp, ibme_pre, ibme_hum);
	sprintf(s, "%s  </bmp>\n",s);
  }
  if(ilux_ok == 1) {
	sprintf(s, "%s  <bh1750>\n",s);
	sprintf(s, "%s   <lux>%f</lux>\n", s, tilux);
	sprintf(s, "%s  </bh1750>\n",s);
  }
  
  sprintf(s, "%s </sensors>\n", s);
  sprintf(s, "%s</esp>\n", s);
  
  return 0;
}

//#pragma GCC pop_options