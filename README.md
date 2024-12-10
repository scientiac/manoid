# A Smart Waste Management Robot using Python and OpenCV

[Manoid (To Understand)](https://scientiac.space/blog/yantra-bot/)

## Basic Setup

### Setting Up Arduino IDE

**To install the ESP32 board in your Arduino IDE, follow these instructions:**

1. In your Arduino IDE, go to File > Preferences.
2. Enter the following into the “Additional Board Manager URLs” field:

```bash

  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

```

3. Click the “OK” button.
4. Open the Boards Manager. Go to Tools > Board > Boards Manager.
5. Search for ESP32 and press the install button for the “ESP32 by Espressif Systems”.
6. Once installed, select your ESP32 board in Tools > Board menu.
7. Select the appropriate Port in Tools > Port menu (in my case it was the `DOIT ESP32 DEVKIT V1`).

Now your Arduino IDE is set up to work with ESP32.

### Setting Up The Environment

I've got my environment all sorted out with Nix and Nix Flake, and I've made it even easier with direnv activation. Mosquitto's up and running smoothly on its default port 1883. `python` and it's dependencies `opencv`,`numpy`,`paho-mqtt` and `flask` in a virtual environment are set up via nix flakes as well. You can look at the [nix documentation](https://nix.dev/) to know more about how it works.

**To set it up:**

1. I assume that nix is installed with flakes enabled on your computer.
2. Clone the repo and enter the environment by running `nix develop` or allowing `direnv` to do it for you if you have it installed.

```bash

  git clone https://github.com/scientiac/manoid

```

3. A MQTT server will be running as soon as you enter the environment on the default port of `1883` and you can check the logs using the `screen` command.
4. Change parameters to match your device and make sure everything in on point.
5. Run the `main.py` script and admire the magic.
