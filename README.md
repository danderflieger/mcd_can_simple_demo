# mcd_can_simple_demo
Sample code to push mock CAN-FiX data to the MakerPlane FiX-Gateway. This just demonstrates basic functionality (e.g. pushing random CAN-FiX messages over the CAN bus network). 

# hardware
- Seeed Studio CANBed - Arduino CAN-Bus Development Kit, ATmega32U4 with Arduino Leonardo Bootloader, MCP2515 and MCP2551 CAN-Bus Controller and Transceiver:
  https://www.amazon.com/gp/product/B0BG7S918T
- BMP280 Temperature/Humidity/Pressure Sensor:
  https://www.amazon.com/dp/B09Q14TBPJ

The BMP280 is connected to the I2C port. Note that some of the BMP280 modules use a different I2C address and you might need to adjust code to connect to the correct address.


