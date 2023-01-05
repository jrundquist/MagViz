#include <Arduino.h>

#include <Adafruit_NeoPixel.h>
#include "AudioTools.h"
#include "AudioLibs/AudioESP32FFT.h" // or any other supported inplementation
#include "BluetoothA2DPSink.h"

#define TOUCH_LIMIT 75
#define UP_PIN T4
#define DOWN_PIN T7
#define NUM_BANDS 8
#define EMAG_PIN A5

uint32_t bands[NUM_BANDS]{/* 0-*/ 63, /* 64-*/ 160, /* 161-*/ 400, /* 401-*/ 1000,
                          /* 1001-*/ 2500, /* 2501-*/ 6250, /* 6250-*/ 16001,
                          /* 16001-*/ 23000};

uint64_t bandMags[NUM_BANDS];

// metroPixel takes in both the number of pixels (1, the built-in) and the pin)
Adafruit_NeoPixel metroPixel = Adafruit_NeoPixel(NEOPIXEL_NUM, PIN_NEOPIXEL);

BluetoothA2DPSink a2dp_sink;
AudioESP32FFT fftc; // or any other supported inplementation

bool isUpTouched = false;
bool isDownTouched = false;
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

  Serial.print(val);
  Serial.print(" ");
  Serial.print(valOn);
  Serial.print(" ");
  Serial.println(" ");
  metroPixel.setPixelColor(0, metroPixel.Color(0x50 * (valOn ? 1 : 0), 0x50 * (valOn ? 1 : 0), 0x50 * (valOn ? 1 : 0)));
  // write the pixel color to the Metro's Neopixel
  metroPixel.show();

  if (!valOn)
  {
    analogWrite(EMAG_PIN, 0x00);
  }
  else
  {
    analogWrite(EMAG_PIN, max(128., min(255., val * 2 * 255)));
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

  Serial.println(bandMags[bin]);
  visualizeIntensity(mult);
}

void setup()
{
  Serial.begin(115200);
  // AudioLogger::instance().begin(Serial, AudioLogger::Info);

  metroPixel.begin();

  pinMode(EMAG_PIN, OUTPUT);

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
  a2dp_sink.set_auto_reconnect(true);
  a2dp_sink.start("MagViz-00");
}

uint8_t pulseNum = 16;
bool isPulseDown = false;
void loop()
{
  delay(100);
  if (!a2dp_sink.is_connected())
  {
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

  if (touchRead(UP_PIN) < TOUCH_LIMIT && !isUpTouched)
  {
    // Touch down
    isUpTouched = true;
    bin = (bin + 1) % NUM_BANDS;
  }
  else if (isUpTouched && touchRead(UP_PIN) > TOUCH_LIMIT)
  {
    // Touch up
    isUpTouched = false;
    Serial.print("Bin: ");
    Serial.println(bin);
  }
  if (touchRead(DOWN_PIN) < TOUCH_LIMIT && !isDownTouched)
  {
    // Touch down
    isDownTouched = true;
    if (bin == 0)
    {
      bin = NUM_BANDS - 1;
    }
    else
    {
      bin = bin - 1;
    }
  }
  else if (isDownTouched && touchRead(DOWN_PIN) > TOUCH_LIMIT)
  {
    // Touch up
    isDownTouched = false;
    Serial.print("Bin: ");
    Serial.println(bin);
  }
}