# Artok HMI Get Started 🚀

This is the **official minimal working example repository** for developers looking to integrate firmware with the **Artok Studio HMI GUI Editor** and its associated Runtime and Flasher tools.

Our goal is simple: **accelerate your hardware development** by providing a clean, pre-configured starting point that links your microcontroller directly to the powerful Artok GUI environment.

---

## ⚡ Quick Start (The 3-Step Integration)

Follow these three steps to successfully flash and run your first Artok HMI project.

### Step 1: Clone and Prepare

Clone this repository to your local machine.

```bash
git clone [https://github.com/Dualturb/artok-get-started.git](https://github.com/Dualturb/artok-get-started.git)
cd artok-get-started
```

### Step 2: Configure and Build
This repository contains the necessary project files for common microcontrollers (e.g., STM32CubeIDE).

1. Open the included project file (e.g., artok-get-started.ioc or similar) in your IDE.

2. Review the main.c file to see how the Artok Runtime is initialized and integrated with the main loop.

3. **Build the project**. This will generate the necessary firmware (.hex or .elf file).

### Step 3: Flash and Run
Use the Artok Studio Flasher tool to deploy the generated firmware.

1. Open the Artok Studio Flasher.

2. Select your target microcontroller (e.g., STM32F103).

3. Specify the path to the newly compiled firmware file (e.g., Debug/artok-get-started.elf).

4. **Flash the device**.

Your device should now initialize the Artok Runtime and be ready to receive and display the GUI designed in the HMI Editor!

## 🛠️ Prerequisites & Key Components

To use this repository effectively, you will need:

| Component | Description | Download Link | 
 | ----- | ----- | ----- | 
| **Artok Studio HMI Editor** | The core tool for designing the user interface and generating the binary HMI data. | [Artok Studio Download](https://artok.dualturb.com) | 
| **Artok Runtime Library** | The pre-compiled library (`libartok_flasher.a`) linked against this firmware, responsible for processing HMI data and managing display/touch events. | Included in the project structure. | 
| **Toolchain** | The correct IDE and compiler (e.g., STM32CubeIDE, PlatformIO) matching the current project. | [IDE/Toolchain](https://www.st.com/en/development-tools/stm32cubeide.html) |

## 🛒 Official Test Hardware
To ensure maximum compatibility and the best out-of-the-box experience, we recommend using the Artok Official Development Board for testing this minimal example.

This hardware is guaranteed to work with the included project configurations.

➡️ [Get the Official Artok Dev Board on eBay](https://www.ebay.com/itm/388985596064)

## 💡 Where to Go Next?
This repository is designed to be a starting line. Once you confirm the base integration works, check out our advanced resources:

- [Artok Studio Editor](https://artok.dualturb.com): Dive deeper into the Artok Runtime API, event handling, and custom drawing functions.

- [Artok Documentation](https://dualturb.com/docs): Dive deeper into the Artok Runtime API, event handling, and custom drawing functions.

- [Advanced Examples Repository](https://github.com/Dualturb):For complex features like sensor integration, data charting, network communication, and multiple hardware targets.

We are committed to making your HMI development seamless and fast! If you encounter any issues, please submit an issue here on GitHub.