#define USE_WIFI 0 //использовать ли MQTT и WIFI
#define MOVING_POINT 1  //влючить эффект падающей точки
#define DEBUG_ENABLE 3 //выводить в порт: 0 - ничего, 1 - сразу после Фурье, 2 - после масштабирования под ленту, 3 - падающая точка

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

//настройки точки
#define timeToFall  300 //время перед падением точки в миллис
#define fallSpeed 150  //скорость падения

//технические настройки матрицы из ленты
#define xres 8  //ширина  (количество столбов) 
#define  yres 6  //высота 
#define trash 600 // шумы
#define numberLeds 48  //количество светодиодов 
#define ledPin 18  //пин ленты 

//настройки звука
#define samplingFrequencyUs 100 //Hz, must be less than 10000 due to ADC
#define noiseFilter 6 //значение шумов (громкость от 0 до 70)
#define micSensetive 50 //чувствительность микрофона 
#define powerKoef 4 // коэфф для повышения чувствительности с ростом частот

//настройки радуги
#define timeFall 100 //время падения столбца
int brightness = 255;  //типа byte (0...255)
int rainbowSpeed = 15;  //скоротсь изменения радуги
int rainbowStep = 10; //шаг радуги между светодиодами

#include "arduinoFFT.h"
#include <driver/adc.h>
#include "FastLED.h"

CRGB leds[numberLeds];  //у меня стока светодиодов

arduinoFFT FFT = arduinoFFT(); //у меня ардуино эквалайзер

int thisMode = 2;  //переменная выбора режима
byte counter = 1;  //счетчик для режима радуги
int rainbowTimer = 0; //и таймер
float smoothValue[xres]; //сглаживание разных громкостей
float averK = 0.006;  //коэф для изменения чувствительности

#if (MOVING_POINT == 1) //бегущая точка
int pointTime[xres];  //отсчет жизни точки
int maxPoint[xres];  //координата максимальной точки
#endif

const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
unsigned long microseconds;
unsigned int samplingPeriod;

//Input vectors receive computed results from FFT
double vReal[samples];
double vImag[samples];
char data_avgs[xres];
int peaks[xres];
int yvalue;
int displayTime[xres];
int displayvalue[xres];
byte color;

#if (USE_WIFI == 1)

#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient wifiClient;
PubSubClient client(wifiClient);
#endif

#if (DEBUG_ENABLE > 0)
#define SERIAL(x) Serial.begin(x)
#define DEBUG(x) Serial.print(x)
#define DEBUGLN(x) Serial.println(x)
#else
#define SERIAL(x)
#define DEBUG(x)
#define DEBUGLN(x)
#endif

void setup() {
  samplingPeriod = round(1000000 * (1.0 / samplingFrequencyUs));

  SERIAL(115200); //запускаю порт
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
  CHSV spectrumcolor;
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
  if ((thisMode == 1 && micros() - microseconds > samplingPeriod)  || (thisMode == 2 && micros() - microseconds > samplingPeriod)) {
    microseconds += samplingPeriod;

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
#if (DEBUG_ENABLE == 1)
    Serial.print((float)data_avgs[0]);
    Serial.print( ", ");
    Serial.print((float)data_avgs[1]);
    Serial.print( ", ");
    Serial.print((float)data_avgs[2]);
    Serial.print( ", ");
    Serial.print((float)data_avgs[3]);
    Serial.print( ", ");
    Serial.print((float)data_avgs[4]);
    Serial.print( ", ");
    Serial.print((float)data_avgs[5]);
    Serial.print( ", ");
    Serial.print((float)data_avgs[6]);
    Serial.print( ", ");
    Serial.println((float)data_avgs[7]);
#endif
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
        //Serial.println(peaks[i]);
      }
      if (yvalue >= peaks[i]) {
        peaks[i] = yvalue ;
        displayTime[i] = millis() + timeFall;
      }
      yvalue = peaks[i];
      displayvalue[i] = yvalue;
    }
#if (DEBUG_ENABLE == 2)
    Serial.print(displayvalue[0]);
    Serial.print( ", ");
    Serial.print(displayvalue[1]);
    Serial.print( ", ");
    Serial.print(displayvalue[2]);
    Serial.print( ", ");
    Serial.print(displayvalue[3]);
    Serial.print( ", ");
    Serial.print(displayvalue[4]);
    Serial.print( ", ");
    Serial.print(displayvalue[5]);
    Serial.print( ", ");
    Serial.print(displayvalue[6]);
    Serial.print( ", ");
    Serial.println(displayvalue[7]);
#endif
  }
  animation();  //расчет столбиков аналайзера
  fallPoint();  //функция падающей точки
  FastLED.show(); //вывести на ленту
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
      }
      if (millis() > rainbowTimer) {
        counter++;
        rainbowTimer = millis() + rainbowSpeed ;
      }
      break;
    case 2:
      for (int k = 0; k < xres; k++) { //для каждого столбца
        //основные столбцы
        for (int i = 0; i < yres; i++ ) {  //для каждой строчки
          color = counter + (i * rainbowStep) + (k * xres) + ((256 / xres) * k);
          if (displayvalue[k] > i && k % 2 == 0) leds[(k * yres) + i] = CHSV(color, 255, 255); //проверяю длину столбца и заливаю радугой
          else if (displayvalue[k] > i && k % 2 == 1) leds[(k * yres) + yres - i - 1] = CHSV(color, 255, 255); //проверяю длину и четность столбца и заливаю радугой
          else if (k % 2 == 0) leds[(k * yres) + i] = CHSV(0, 255, 0);  //остальные крашу в черный
          else if (k % 2 == 1) leds[(k * yres) + yres - i - 1] = CHSV(0, 255, 0);  //остальные крашу в черный
          //Serial.println(displayvalue[1]);
        }
      }
      if (millis() > rainbowTimer) {
        counter++;
        rainbowTimer = millis() + rainbowSpeed ;
      }
      break;
  }
}

void fallPoint () { //бегущая точка
#if (MOVING_POINT == 1)
  for (int g = 0; g < xres; g++) {
    if (displayvalue[g] >= maxPoint[g]) {
      maxPoint[g] = displayvalue[g];
      pointTime[g] = millis() + timeToFall;
    } else if (pointTime[g] < millis() && maxPoint[g] > 0) {
      maxPoint[g] = maxPoint[g] - 1;
      pointTime[g] = millis() + fallSpeed;
    }
  }
  for (int j = 0; j < xres; j = j + 2) {
    leds[(j * yres) + maxPoint[j]] = CHSV(255, 255, 255);
  }
  for (int h = 1; h < xres; h = h + 2) {
    leds[(h * yres) + yres - maxPoint[h] - 1] = CHSV(255, 255, 255);
  }
#endif
#if (DEBUG_ENABLE == 3)
  Serial.print(maxPoint[0]);
  Serial.print( ", ");
  Serial.print(maxPoint[1]);
  Serial.print( ", ");
  Serial.print(maxPoint[2]);
  Serial.print( ", ");
  Serial.print(maxPoint[3]);
  Serial.print( ", ");
  Serial.print(maxPoint[4]);
  Serial.print( ", ");
  Serial.print(maxPoint[5]);
  Serial.print( ", ");
  Serial.print(maxPoint[6]);
  Serial.print( ", ");
  Serial.println(maxPoint[7]);
#endif
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
    DEBUG("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "soundStelaj";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      DEBUGLN("connected");
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
void callback(char* topic, byte * payload, unsigned int lengthPackage) {
  if (String(topic) == "/ESP32/audioStelaj/brightness") { //меняется ручками значение канала, дефайн не меняет скобочки
    brightness = map(payload[0], 48, 57, 0, 255); //настройка яркости подсветки
    FastLED.setBrightness(brightness);
  }
  if (String(topic) == "/ESP32/audioStelaj/thisMode") {
    thisMode = 0;
    for (int i = 0; i < lengthPackage; i++) {
      thisMode = thisMode + ((payload[i] - 48) * pow(10, (lengthPackage - i - 1))); //выбор режима подсветки
    }
    DEBUG("Got mode: ");
    DEBUGLN(thisMode);
  }
  if (String(topic) == "/ESP32/audioStelaj/rainbowSpeed") {
    rainbowSpeed = 0;
    for (int i = 0; i < lengthPackage; i++) {
      rainbowSpeed = rainbowSpeed + ((payload[i] - 48) * pow(10, (lengthPackage - i - 1))); //выбор режима подсветки
    }
    DEBUG("Got rainbow speed: ");
    DEBUGLN(rainbowSpeed);
  }
  if (String(topic) == "/ESP32/audioStelaj/rainbowStep") {
    rainbowStep = 0;
    for (int i = 0; i < lengthPackage; i++) {
      rainbowStep = rainbowStep + ((payload[i] - 48) * pow(10, (lengthPackage - i - 1))); //выбор режима подсветки
    }
    DEBUG("Got rainbow step: ");
    DEBUGLN(rainbowStep);
  }
}
void publishSerialData(char *serialData) {
  if (!client.connected()) {
    reconnect();
  }
  client.publish(MQTT_SERIAL_PUBLISH_CH, serialData);
}
#endif
