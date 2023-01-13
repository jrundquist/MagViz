# MagViz

Audio Visualization using an ESP32. 

## Functionality
Creates a cool visual effect in the ferrofluid using an electromagnet.

### Line In

* When a line in signal is present, the MSGEQ7 is used to extract the frequency
  bands. 
* When the delta betwen the current and previous 'bass' signal is great enough,
  the electromagnet is enabled, as long as the signal is rising, the
  electromagnet remains on.

### Bluetooth 

* When enabled (`#define ENABLE_BLUETOOTH true`), allows the ESP32 to connect as
  a bluetooth audio client
* Uses the AudioLibs FFT to compute the 'bass'
* Using the bass signal, enables the electromagnet when the base is high enough


## Hardware

* 12v 1amp power supply.
* [ESP32 Feather
  V2](https://learn.adafruit.com/adafruit-esp32-feather-v2/overview)
* [12v Electromagnet](https://www.amazon.com/dp/B01N3386NB?psc=1&ref=ppx_yo2ov_dt_b_product_details)
* [TSR 1-2450 step-down switching regulator
  5v](https://www.digikey.com/en/products/detail/traco-power/TSR-1-2450/9383780?utm_adgroup=Converters&utm_source=google&utm_medium=cpc&utm_campaign=Dynamic%20Search_EN_RLSA_Buyers&utm_term=&utm_content=Converters&gclid=Cj0KCQiA_P6dBhD1ARIsAAGI7HCfzF7Vk7mN3mUC-9isVxN6BnOa7UUd-CddQZrVGMTIzRErmd-__44aAihSEALw_wcB)
* [Ferrofluid](https://www.amazon.com/dp/B09JBJ7DVV?psc=1&ref=ppx_yo2ov_dt_b_product_details)
* MSGEQ7 in the form of [Audio Analyzer
  Module](https://www.dfrobot.com/product-514.html?gclid=CjwKCAiAh9qdBhAOEiwAvxIokybAUt4c6UMCYhUnveRxxAq_f0AWr6-sIURl1Z7LSwjBIOeOHorsOBoC85AQAvD_BwE) 
* [RCA Female
  Breakouts](https://www.amazon.com/dp/B0BJTFVPSR?psc=1&ref=ppx_yo2ov_dt_b_product_details)
