# TTGO-LoRa32-V1.0-TTN-OTAA
Example project to connect the TTGO-LoRa32-V1.0 to TheThingsNetwork over OTAA

You can find the setup instructions for TTN V3 here: https://blog.squix.org/2021/07/ttgo-lora32-v1-0-with-ttn-v3-and-otaa.html

You can buy the device here: https://blog.squix.org/go/bg-ttgo-lora32-v1


# ESP32 and SX1278 Modbus Data Logger

This project combines an ESP32 microcontroller with an SX1278 LoRa module to create a versatile Modbus data logger. It is capable of reading data from a Modbus server, transmitting that data over LoRa to a designated server, and also providing a WiFi-based web server for real-time monitoring and control.

## Hardware Requirements

- **ESP32 Development Board:** This is the heart of the project, responsible for controlling all operations.

- **SX1278 LoRa Module:** The LoRa module is used for long-range wireless communication.

- **ENC28J60 Ethernet Module:** This module enables Ethernet connectivity for communication with other devices and the web server.

- **Modbus Server:** You'll need a Modbus server, such as a PLC or industrial controller, to provide the data you want to log.

## Software Requirements

- **Arduino IDE:** The project is developed using the Arduino IDE.

- **LMIC Library:** For LoRa communication, this library is essential.

- **WebServer_ESP32_ENC Library:** It provides support for Ethernet connectivity and the web server.

## Installation

1. **Clone or Download the Repository:** Start by obtaining the project files from this repository.

2. **Open the Arduino IDE:** Launch the Arduino IDE on your computer.

3. **Install Required Libraries:** If you haven't already, install the necessary libraries, namely `LMIC` and `WebServer_ESP32_ENC`.

4. **Hardware Setup:** Connect the hardware components as described in your code. Pay attention to pin connections and wiring.

## Configuration

1. **Modify Code Settings:** Customize the following placeholders in the code with your specific settings:

   - **WiFi SSID and Password:** Enter the credentials for your WiFi network.

   - **LoRa Settings:** Configure the LoRa module settings, such as frequency and keys.

   - **Modbus Server:** Specify the Modbus server's IP address and port.

   - **MAC Addresses:** Enter the MAC addresses of your devices.

   - **Modbus Addressing:** Configure the Modbus addresses according to your setup.

2. **Upload the Code:** Upload the modified code to your ESP32 board using the Arduino IDE.

## Usage

1. **Connect to WiFi:** Ensure that your ESP32 is connected to your WiFi network.

2. **Modbus Server:** Verify that the Modbus server is running and accessible at the specified IP and port.

3. **LoRa Module:** Power on the LoRa module and confirm that it's set to the correct frequency band.

4. **Data Logging:** The ESP32 will periodically read data from the Modbus server and transmit it to the designated LoRa server.

## Web Server

The ESP32 also hosts a web server that can be accessed through your WiFi network. This web interface allows you to monitor and control the device. To access the web server, open a web browser and enter the IP address of your ESP32.

## Troubleshooting

- **Serial Monitor:** If you encounter issues, check the Arduino IDE's serial monitor for debugging information.

- **Hardware Connections:** Verify that the hardware components are correctly connected.

- **Modbus Server:** Ensure that the Modbus server is operational and reachable.

## Credits

This project relies on the [LMIC-Arduino](https://github.com/matthijskooijman/arduino-lmic) and [WebServer_ESP32_ENC](https://github.com/jgiacomini/WebServer_ESP32_ENC) libraries for its core functionality.

## License

This project is licensed under the MIT License. You can find more details in the [LICENSE](LICENSE) file.
