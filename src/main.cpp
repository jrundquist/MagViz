#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <vector>

#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT
// #define ONBOARD_AUDIO
#else
#define SHOW_GRAPHIC
#endif

#ifdef ONBOARD_AUDIO
#include "AudioTools.h"
#include "AudioLibs/AudioESP32FFT.h" // or any other supported inplementation
#include "BluetoothA2DPSink.h"
#endif

#ifdef SHOW_GRAPHIC
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
#endif

#include "MSGEQ7.h"

#define NUM_BANDS 8
#define EMAG_PIN A5

#define MSG_OUT A0
#define MSG_RST MOSI
#define MSG_READ MISO
#define MSGEQ7_INTERVAL ReadsPerSecond(10)
#define MSGEQ7_SMOOTH 100 // Range: 0-255 -- 191

#if defined(NEOPIXEL_NUM) && defined(PIN_NEOPIXEL)
#define USE_PIXEL true
#endif

#ifdef USE_PIXEL
// metroPixel takes in both the number of pixels (1, the built-in) and the pin)
Adafruit_NeoPixel metroPixel = Adafruit_NeoPixel(NEOPIXEL_NUM, PIN_NEOPIXEL);
#endif

CMSGEQ7<MSGEQ7_SMOOTH, MSG_RST, MSG_READ, MSG_OUT> MSGEQ7;

#ifdef ONBOARD_AUDIO
uint32_t bands[NUM_BANDS]{/* 0-*/ 63, /* 64-*/ 160, /* 161-*/ 400, /* 401-*/ 1000,
                          /* 1001-*/ 2500, /* 2501-*/ 6250, /* 6250-*/ 16001,
                          /* 16001-*/ 23000};

uint64_t bandMags[NUM_BANDS];

BluetoothA2DPSink a2dp_sink;
AudioESP32FFT fftc; // or any other supported inplementation
uint64_t bin = 0;

// Provide data to FFT
void writeDataStream(const uint8_t *data, uint32_t length)
{
  fftc.write(data, length);
}

void visualizeIntensity(double amt)
{

  auto val = bandMags[bin] / 1000000.0;
  auto valOn = val > 0.25;

  // Serial.print(val);
  // Serial.print(" ");
  // Serial.print(valOn);
  // Serial.print(" ");
  // Serial.println(" ");
#ifdef USE_PIXEL
  metroPixel.setPixelColor(0, metroPixel.Color(0x50 * (valOn ? 1 : 0), 0x50 * (valOn ? 1 : 0), 0x50 * (valOn ? 1 : 0)));
  // write the pixel color to the Metro's Neopixel
  metroPixel.show();
#endif

  if (!valOn)
  {
    digitalWrite(EMAG_PIN, LOW);
  }
  else
  {
    digitalWrite(EMAG_PIN, HIGH);
  }
}

// display fft result
void fftResult(AudioFFTBase &fft)
{
  auto result = fftc.result();
  if (result.magnitude <= 100)
  {
    visualizeIntensity(0);
    return;
  }

  // Clear the bandMags
  for (int i = 0; i < NUM_BANDS; i++)
  {
    bandMags[i] = 0;
  }
  uint64_t totalMag = 0;

  // Compute the bands from the FFT results
  int band = 0;
  for (int i = 0; i < fft.size(); i++)
  {
    const auto freq = fft.frequency(i);
    while (freq > bands[band])
    {
      band++;
    }
    const auto mag = fft.magnitude(i);
    bandMags[band] += uint64_t(mag / 100);
    totalMag += uint64_t(mag / 100);
  }

  // Determine the maximum band
  int maxBin = 0;
  for (int i = 0; i < NUM_BANDS - 1; i++)
  {
    if (bandMags[i] > bandMags[maxBin])
    {
      maxBin = i;
    }
  }

  // Output the
  auto mult = max(0.0, min(1.0, bandMags[bin] / double(totalMag)));

  // Serial.println(bandMags[bin]);
  visualizeIntensity(mult);
}
#endif

void setup()
{
  Serial.begin(115200);
  // AudioLogger::instance().begin(Serial, AudioLogger::Info);

  // Setup MSGEQ7
  MSGEQ7.begin();

#ifdef USE_PIXEL
  // Setup Metro Pixel
  metroPixel.begin();
#endif

#ifdef SHOW_GRAPHIC
  Serial.println("Screen");

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  tft.init(135, 240); // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_ORANGE);
  tft.setTextSize(3);
  tft.setCursor(0, 0);
  tft.println("Starting...");
#endif

  // Setup EMAG
  pinMode(EMAG_PIN, OUTPUT);

#ifdef ONBOARD_AUDIO
  // Setup FFT
  auto tcfg = fftc.defaultConfig();
  tcfg.length = 4096;
  tcfg.channels = 2;
  tcfg.sample_rate = a2dp_sink.sample_rate();
  tcfg.bits_per_sample = 16;
  tcfg.callback = &fftResult;
  fftc.begin(tcfg);

  // register callback
  a2dp_sink.set_stream_reader(writeDataStream, false);

  // Start Bluetooth Audio Receiver
  Serial.println("starting a2dp-fft...");
  a2dp_sink.set_auto_reconnect(false);
  a2dp_sink.start("MagViz-00");
#endif
}

// Print the current values to the serial monitor
void serialBars(uint16_t FPS)
{
  // Visualize the average bass of both channels
  uint8_t input = MSGEQ7.get(MSGEQ7_BASS);

  // Reduce noise
  input = mapNoise(input);

  // Save the difference between the last beat
  static uint8_t lastInput = 0;
  int delta = input - lastInput;
  lastInput = input;

  // All channels together
  Serial.print(input);
  Serial.print(F(" In\t"));

  // Difference between last measurement
  Serial.print(delta);
  Serial.print(F(" D\t"));

  // 1st channel (here: left)
  Serial.print(MSGEQ7.get(MSGEQ7_BASS, 0));
  Serial.print(F(" R\t"));

  // 2nd channel (here: left)
  Serial.print(MSGEQ7.get(MSGEQ7_BASS, 1));
  Serial.print(F(" L\t"));

  // overall volume of all channels
  Serial.print(MSGEQ7.getVolume());
  Serial.print(F(" Vol\t"));

  // FPS of the reading
  Serial.print(FPS);
  Serial.print(F(" FPS \t"));

  // Highlight high pitches
  char c = '=';
  if (delta >= 20)
    c = '#';

  // Visualize with characters as bars
  for (uint8_t i = 0; i < (input / 4); i++)
  {
    if (i == 192 / 4 - 1)
      Serial.print('+');
    else if (i == 128 / 4 - 1)
      Serial.print('*');
    else if (i == 64 / 4 - 1)
      Serial.print('X');
    else
      Serial.print(c);
  }
  Serial.println();
}

uint16_t getFPS(bool newReading)
{
  // Variables to count FPS and last 1 second mark
  static uint16_t prevFPS = 0;
  static uint16_t FPS = 0;
  static uint32_t m = 0;

  // Increase FPS count
  if (newReading)
    FPS++;

  // If 1 second mark crossed, save new FPS
  if ((micros() - m) > 1000000)
  {
    prevFPS = FPS;
    FPS = 0;
    m = micros();
  }

  return prevFPS;
}

void displayEqualizer(uint16_t FPS)
{
  int width = tft.width() / (NUM_BANDS - 1);
  std::vector<std::tuple<int, uint32_t>> bands = {
      std::make_tuple(MSGEQ7_0, ST77XX_RED),
      std::make_tuple(MSGEQ7_1, ST77XX_ORANGE),
      std::make_tuple(MSGEQ7_2, ST77XX_YELLOW),
      std::make_tuple(MSGEQ7_3, ST77XX_GREEN),
      std::make_tuple(MSGEQ7_4, ST77XX_BLUE),
      std::make_tuple(MSGEQ7_5, ST77XX_MAGENTA),
      std::make_tuple(MSGEQ7_6, ST77XX_WHITE /* Purple */),
  };
  for (int i = 0; i < bands.size(); i++)
  {
    int band = std::get<0>(bands[i]);
    uint32_t color = std::get<1>(bands[i]);
    Serial.println(MSGEQ7.getVolume());
    float height = -(float(MSGEQ7.get(band)) / 255.0) * (tft.height() - 5);
    tft.fillRect(i * width, 0, width, int((tft.height() - 5) + height), ST77XX_BLACK);
    tft.fillRect(i * width, (tft.height() - 5), width, int(height), color);
  }
  delay(0);
}

uint8_t pulseNum = 16;
bool isPulseDown = false;
bool isBluetooth = false;

void loop()
{
  // Analyze without delay every interval
  bool newReading = MSGEQ7.read(MSGEQ7_INTERVAL);

  // Calculate FPS
  uint16_t FPS = getFPS(newReading);

  // Serial raw debug output
  if (newReading && !isBluetooth)
  {
#ifdef SHOW_GRAPHIC
    displayEqualizer(FPS);
#endif
    // serialBars(FPS);

    // Visualize the average bass of both channels
    uint8_t input = MSGEQ7.get(MSGEQ7_BASS);

    // Reduce noise
    input = mapNoise(input);

    // Save the difference between the last beat
    static uint8_t lastInput = 0;
    int delta = input - lastInput;
    lastInput = input;

    // Save the emag on state;
    static bool isOn = false;

    if (!isOn && delta > 20 || isOn && delta > 0)
    {
#ifdef USE_PIXEL
      metroPixel.setPixelColor(0, metroPixel.Color(0xb0, 0x53, 0x00));
      // write the pixel color to the Metro's Neopixel
      metroPixel.show();
#endif
      digitalWrite(EMAG_PIN, HIGH);
      isOn = true;
    }
    else
    {
#ifdef USE_PIXEL
      metroPixel.setPixelColor(0, metroPixel.Color(0x00, 0x01, 0x00));
      // write the pixel color to the Metro's Neopixel
      metroPixel.show();
#endif
      digitalWrite(EMAG_PIN, LOW);
      isOn = false;
    }
    return;
  }

#ifdef ONBOARD_AUDIO
  if (!a2dp_sink.is_connected())
  {
    isBluetooth = false;
    if (!isPulseDown)
    {
      pulseNum += 16;
    }
    else
    {
      pulseNum -= 16;
    }

    if (pulseNum == 16 || pulseNum == 240)
      isPulseDown = !isPulseDown;

#ifdef USE_PIXEL
    metroPixel.setPixelColor(0, metroPixel.Color(0x00, 0x00, pulseNum / 4));
    metroPixel.show();
#endif
  }
  else
  {
    isBluetooth = true;
  }
#endif
}