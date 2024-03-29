# mcd_can_simple_demo
This repository contains sample code that pushes mock CAN-FiX data to the MakerPlane FiX-Gateway. This just demonstrates basic functionality (e.g. pushing random CAN-FiX messages over the CAN bus network). There is one exception: It also reads sensor data from a BMP280 temperature/humidity/barometric pressure sensor module.

I have added a LOT of comment lines to describe what is happening. If you have questions, it's probably already explained in the commented sections.

# Hardware
- Seeed Studio CANBed - Arduino CAN-Bus Development Kit, ATmega32U4 with Arduino Leonardo Bootloader, MCP2515 and MCP2551 CAN-Bus Controller and Transceiver:
  https://www.amazon.com/gp/product/B0BG7S918T
- BMP280 Temperature/Humidity/Pressure Sensor:
  https://www.amazon.com/dp/B09Q14TBPJ

The BMP280 is connected to the I2C port. Note that some of the BMP280 modules use a different I2C address and you might need to adjust code to connect to the correct address.

# Required Libraries
- mcp_can - In the Arduino IDE's library manager, search for **mcp_can** by coryjfowler. This is the library used to communicate with the MCP2515 CAN module
- canfix - This is a library available from MakerPlane: https://github.com/makerplane/CAN-FIX-ArduinoLib
