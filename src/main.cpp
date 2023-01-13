#include <Arduino.h>

#include <Adafruit_NeoPixel.h>
#include "AudioTools.h"
#include "AudioLibs/AudioESP32FFT.h" // or any other supported inplementation
#include "BluetoothA2DPSink.h"
#include "MSGEQ7.h"

#define ENABLE_BLUETOOTH false

#define TOUCH_LIMIT 75
#define UP_PIN T4
#define DOWN_PIN T7
#define NUM_BANDS 8
#define EMAG_PIN A5

#define pinAnalogLeft A0
#define pinAnalogRight A0
#define pinReset 19
#define pinStrobe 21
#define MSGEQ7_INTERVAL ReadsPerSecond(50)
#define MSGEQ7_SMOOTH 191 // Range: 0-255

// metroPixel takes in both the number of pixels (1, the built-in) and the pin)
Adafruit_NeoPixel metroPixel = Adafruit_NeoPixel(NEOPIXEL_NUM, PIN_NEOPIXEL);

CMSGEQ7<MSGEQ7_SMOOTH, pinReset, pinStrobe, pinAnalogLeft> MSGEQ7;

#if ENABLE_BLUETOOTH == true
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
  metroPixel.setPixelColor(0, metroPixel.Color(0x50 * (valOn ? 1 : 0), 0x50 * (valOn ? 1 : 0), 0x50 * (valOn ? 1 : 0)));
  // write the pixel color to the Metro's Neopixel
  metroPixel.show();

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

  // For debugging purposes, display the band magnitudes
  // for (int i = 0; i < NUM_BANDS; i++)
  // {

  //   Serial.print(bandMags[i] / double(totalMag));
  //   Serial.print(" ");
  // }

  // Display the max band
  // Serial.print(" -- ");
  // Serial.print(maxBin);
  // Serial.println("");

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

  MSGEQ7.begin();

  metroPixel.begin();

  pinMode(EMAG_PIN, OUTPUT);

#if ENABLE_BLUETOOTH == true
  // Setup FFT
  auto tcfg = fftc.defaultConfig();
  tcfg.length = 4096;
  tcfg.channels = 2;
  tcfg.sample_rate = a2dp_sink.sample_rate();
  ;
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
    // Print the bars
    // serialBars(newReading);

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
      metroPixel.setPixelColor(0, metroPixel.Color(0x00, 0x03, 0x00));
      // write the pixel color to the Metro's Neopixel
      // metroPixel.show();
      digitalWrite(EMAG_PIN, HIGH);
      isOn = true;
    }
    else
    {
      metroPixel.setPixelColor(0, metroPixel.Color(0x00, 0x01, 0x00));
      // write the pixel color to the Metro's Neopixel
      metroPixel.show();
      digitalWrite(EMAG_PIN, LOW);
      isOn = false;
    }
    return;
  }

#if ENABLE_BLUETOOTH == true
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

    metroPixel.setPixelColor(0, metroPixel.Color(0x00, 0x00, pulseNum / 4));
    metroPixel.show();
  }
  else
  {
    isBluetooth = true;
  }
#endif
}