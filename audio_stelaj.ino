#define USE_WIFI 0 //использовать ли MQTT и WIFI

#if (USE_WIFI == 1)
//настройки WIFI и MQTT
const char* ssid = "FSOCIETY";
const char* password = "WannaHackMe";
const char* mqtt_server = "192.168.0.34";
#define mqtt_port 1883
#define MQTT_USER "datmon"
#define MQTT_PASSWORD "amabof33"
#define MQTT_SERIAL_PUBLISH_CH "/ESP32/audioStelaj/tx"
#define MQTT_SERIAL_RECEIVER_brightness "/ESP32/audioStelaj/brightness"
#define MQTT_SERIAL_RECEIVER_thisMode "/ESP32/audioStelaj/thisMode"
#define MQTT_SERIAL_RECEIVER_rainbowSpeed "/ESP32/audioStelaj/rainbowSpeed"
#define MQTT_SERIAL_RECEIVER_rainbowStep "/ESP32/audioStelaj/rainbowStep"

#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient wifiClient;
PubSubClient client(wifiClient);
#endif

//технические настройки матрицы из ленты
#define xres 8  //ширина  (количество столбов) 
#define  yres 6  //высота 
#define trash 600 // шумы
#define numberLeds 48  //количество светодиодов 
#define ledPin 18  //пин ленты 

//настройки звука
#define noiseFilter 6 //значение шумов (громкость от 0 до 70)
#define micSensetive 50 //чувствительность микрофона 
#define powerKoef 4 // коэфф для повышения чувствительности с ростом частот

//настройки радуги
#define timeFall 100 //время падения столбца
int brightness = 255;  //типа byte (0...255)
int rainbowSpeed = 15;  //скоротсь изменения радуги
int rainbowStep = 10; //шаг радуги между светодиодами

//настройки точки
//#define timeToFall  1000 //время перед падением точки в миллис
//#define fallSpeed 500  //скорость падения

#include "arduinoFFT.h"
#include <driver/adc.h>
#include "FastLED.h"

CRGB leds[numberLeds];  //у меня стока светодиодов

arduinoFFT FFT = arduinoFFT(); //у меня ардуино эквалайзер

int thisMode = 1;  //переменная выбора режима
byte counter = 1;  //счетчик для режима радуги
int rainbowTimer = 0; //и таймер
float smoothValue[xres]; //сглаживание разных громкостей
float averK = 0.006;  //коэф для изменения чувствительности

//бегущая точка
//byte antiCounter = 128; //противоположный цвет радуги для бегущей точки
//int pointTime[xres];  //отсчет жизни точки
//int maxPoint[xres];  //координата максимальной точки
//int fallOut[xres];

const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
//const double samplingFrequency = 10000; //Hz, must be less than 10000 due to ADC

//Input vectors receive computed results from FFT
double vReal[samples];
double vImag[samples];
char data_avgs[xres];
int peaks[xres];
int yvalue;
int displayTime[xres];
int displayvalue[xres];
byte color;

void setup() {
  Serial.begin(115200);
#if (USE_WIFI == 1)
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();
  Serial.setTimeout(500); // Set time out for wifi
#endif

  //настройка порта
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
  pinMode(5, INPUT);

  //лента
  FastLED.addLeds<WS2812B, ledPin, GRB>(leds, numberLeds).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(brightness);
  pinMode(18, OUTPUT);
}

void loop() {
#if (USE_WIFI == 1)
  client.loop();  //функция для MQTT

  //читаю терминал
  if (Serial.available() > 0) {
    char mun[501];
    memset(mun, 0, 501);
    Serial.readBytesUntil( '\n', mun, 500);
    publishSerialData(mun);
  }
#endif
  //выбираю режим
  if (thisMode == 1) {
    /*SAMPLING*/
    for (int i = 0; i < samples; i++)
    {
      vReal[i] = (adc1_get_raw(ADC1_CHANNEL_7) - trash) / 8;
      vImag[i] = 0;
      //Serial.println(adc1_get_raw(ADC1_CHANNEL_7) - trash);
    }
    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */

    //double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
    // Serial.println(x, 6); //Print out what frequency is the most dominant.

    int steps = (samples / 2) / xres; //получилось 4
    int c = 0;
    /*
      Serial.print(vReal[10]);
      Serial.print( ", ");
      Serial.println(vReal[40]);
    */
    for (int i = 4; i <= (samples / 2); i += steps)  {
      data_avgs[c] = 0;
      for (int k = 0 ; k < steps ; k++) {
        data_avgs[c] = data_avgs[c] + vReal[i + k];
      }
      data_avgs[c] = data_avgs[c] / steps;
      c++;
    }
    //симуляция звука
    /*
      if (digitalRead(5)) {
      for (int i; i < xres; i++) {
        peaks[i] = yres;
      }
      }
      /*
      Serial.print("Button pressed: ");
      Serial.print(displayvalue[xres - 1]);
      Serial.println(" has been sent");
    */
    /*
      Serial.print((float)data_avgs[1]);
      Serial.print( ", ");
      Serial.print((float)data_avgs[3]);
      Serial.print( ", ");
      Serial.println((float)data_avgs[7]);
    */
    for (int i = 0; i < xres; i++)
    {
      if (data_avgs[i] < noiseFilter) data_avgs[i] = 0; //отсекаю низжний порог шумов
      smoothValue[i] = (float) data_avgs[i] * averK + smoothValue[i] * (1 - averK); //высчитываю переменную для адаптации под громкость
      //Serial.println(smoothValue[0]);
      data_avgs[i] = constrain(data_avgs[i], noiseFilter, micSensetive - (i * powerKoef) + smoothValue[i]); //ограничиваю массив
      data_avgs[i] = map(data_avgs[i], noiseFilter, micSensetive - (i * powerKoef) + smoothValue[i], 0, yres);  //маштабирую массив под размеры матрицы
      yvalue = data_avgs[i];
      if (millis() > displayTime[i] && peaks[i] != 0) { //плавное падение
        displayTime[i] = millis() + timeFall;
        peaks[i] = peaks[i] - 1;    // decay by one light
        Serial.println(peaks[i]);
      }
      if (yvalue >= peaks[i]) {
        peaks[i] = yvalue ;
        displayTime[i] = millis() + timeFall;
      }
      yvalue = peaks[i];
      displayvalue[i] = yvalue;
    }
    ///*
    Serial.print(displayvalue[5]);
    Serial.print( ", ");
    Serial.print(displayvalue[6]);
    Serial.print( ", ");
    Serial.println(displayvalue[7]);
    //*/
  }
  animation();
}
void animation() {
  switch (thisMode) {
    case 0:
      for (int i = 0; i < numberLeds; i++) {
        leds[i] = CHSV(255, 255, 255);
      }
      break;
    case 1:
      for (int k = 0; k < xres; k++) { //для каждого столбца
        //основные столбцы
        for (int i = 0; i < yres; i++ ) {  //для каждой строчки
          color = counter + (i * rainbowStep) + (k * xres);
          if (displayvalue[k] > i && k % 2 == 0) leds[(k * yres) + i] = CHSV(color, 255, 255); //проверяю длину столбца и заливаю радугой
          else if (displayvalue[k] > i && k % 2 == 1) leds[(k * yres) + yres - i - 1] = CHSV(color, 255, 255); //проверяю длину и четность столбца и заливаю радугой
          else if (k % 2 == 0) leds[(k * yres) + i] = CHSV(0, 255, 0);  //остальные крашу в черный
          else if (k % 2 == 1) leds[(k * yres) + yres - i - 1] = CHSV(0, 255, 0);  //остальные крашу в черный
          //Serial.println(displayvalue[1]);
        }
        /*
          Serial.print(rainbowSpeed);
          Serial.print(", ");
          Serial.println(rainbowStep);
        */

        /*//бегущая точка
          if (displayvalue[k] > maxPoint[k]) {  //если есть значение в столбе
          maxPoint[k] = displayvalue[k] + (k * yres);  //добавляю макспоинт
          leds[maxPoint[k]] = CHSV(antiCounter, 255, 255);
          pointTime[k] = millis() + timeToFall;  //время появления точки
          Serial.println("born");
          } else if (pointTime[k] > millis()) {  //если точка свое не изжила без столба
          leds[maxPoint[k]] = CHSV(antiCounter, 255, 255);
          Serial.println(k);
          } else if (pointTime[k] < millis() && fallOut[k] < millis() && maxPoint[k] > yres * k) {
          leds[maxPoint[k]] = CHSV(antiCounter, 0, 0);
          maxPoint[k] = maxPoint[k] - 1;
          leds[maxPoint[k]] = CHSV(antiCounter, 255, 255);
          fallOut[k] = millis() + fallSpeed;
          Serial.println("falling");
          } else if (maxPoint[k] = yres * k) {
          fallOut[k] = 0;
          leds[maxPoint[k]] = CHSV(antiCounter, 0, 0);
          maxPoint[k] = 0;
          pointTime[k] = 0;
          Serial.println("death");
          }
        */
      }
      FastLED.show();
      if (millis() > rainbowTimer) {
        counter++;
        //antiCounter++;
        rainbowTimer = millis() + rainbowSpeed ;
      }
      break;
  }
}
#if (USE_WIFI == 1)
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { //попытка подключения к WIFI
    delay(500);
  }
  randomSeed(micros());
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "soundStelaj";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/icircuit/presence/ESP32/", "hello world");
      // ... and resubscribe
      client.subscribe(MQTT_SERIAL_RECEIVER_brightness);
      client.subscribe(MQTT_SERIAL_RECEIVER_thisMode);
      client.subscribe(MQTT_SERIAL_RECEIVER_rainbowSpeed);
      client.subscribe(MQTT_SERIAL_RECEIVER_rainbowStep);
    } else {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void callback(char* topic, byte *payload, unsigned int length) {
  if (String(topic) == "/ESP32/audioStelaj/brightness") { //меняется ручками значение канала, дефайн не меняет скобочки
    brightness = map(payload[0], 48, 57, 0, 255); //настройка яркости подсветки
    FastLED.setBrightness(brightness);
  }
  if (String(topic) == "/ESP32/audioStelaj/thisMode") thisMode = payload[0] - 48; //выбор режима подсветки
  if (String(topic) == "/ESP32/audioStelaj/rainbowSpeed") rainbowSpeed = map(payload[0] - 48, 0, 9, 0, 20);
  if (String(topic) == "/ESP32/audioStelaj/rainbowStep") rainbowStep = map(payload[0] - 48, 0, 9, 0, 20);
  /*
    Serial.println("-------new message from broker-----");
    Serial.print("channel:");
    Serial.println(topic);
    Serial.print("data:");
    Serial.write(payload, length);
    Serial.println();
    if ((char)payload[0] == '0') Serial.println("it's wotking!!!");
  */
}
void publishSerialData(char *serialData) {
  if (!client.connected()) {
    reconnect();
  }
  client.publish(MQTT_SERIAL_PUBLISH_CH, serialData);
}
#endif