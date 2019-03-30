/*!
 *
 *  @file Adafruit_STMPE610.cpp
 *
 *  @mainpage Adafruit STMPE610 Resistive Touch Screen Controller
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the Adafruit STMPE610 Resistive
 *  touch screen controller breakout
 *  ----> http://www.adafruit.com/products/1571
 *
 *  Check out the links above for our tutorials and wiring diagrams
 *  These breakouts use SPI or I2C to communicate
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 *  @section author Author
 *
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 *
 *  Updated to work with Teensy 3.6 by Timo Sandmann
 */

#include "Adafruit_STMPE610.h"


/*!
 *  @brief  Instantiates a new STMPE610 class using bitbang SPI
 *  @param  cspin
 *          CS pin
 *  @param  mosipin
 *          MOSI pin
 *  @param  misopin
 *          MISO pin
 *  @param  clkpin
 *          CLK pin
 */
Adafruit_STMPE610::Adafruit_STMPE610(uint8_t cspin, uint8_t mosipin, uint8_t misopin, uint8_t clkpin)
    : _wire {}, _spi {}, _CS { static_cast<int8_t>(cspin) }, _MOSI { static_cast<int8_t>(mosipin) }, _MISO { static_cast<int8_t>(misopin) },
      _CLK { static_cast<int8_t>(clkpin) }, _i2caddr {}, _spiMode {} {}

/*!
 *  @brief  Instantiates a new STMPE610 using provided SPI
 *  @param  cspin
 *          CS pin
 *  @param  *theSPI
 *          spi object
 */
Adafruit_STMPE610::Adafruit_STMPE610(uint8_t cspin, SPIClass* theSPI)
    : _wire {}, _spi { theSPI }, _CS { static_cast<int8_t>(cspin) }, _MOSI { -1 }, _MISO { -1 }, _CLK { -1 }, _i2caddr {}, _spiMode {} {}

/*!
 *  @brief  Instantiates a new STMPE610 using provided Wire
 *  @param  *theWire
 *          wire object
 */
Adafruit_STMPE610::Adafruit_STMPE610(TwoWire* theWire)
    : _wire { theWire }, _spi {}, _CS { -1 }, _MOSI { -1 }, _MISO { -1 }, _CLK { -1 }, _i2caddr {}, _spiMode {} {}

/*!
 *  @brief  Setups the HW
 *  @param  i2caddr
 *          I2C address (defaults to STMPE_ADDR)
 *  @return True if process is successful
 */
bool Adafruit_STMPE610::begin(uint8_t i2caddr) {
    if (_CS != -1 && _CLK == -1) {
        // hardware SPI
        pinMode(_CS, OUTPUT);
        digitalWriteFast(_CS, HIGH);

        _spi->begin();
        _spisettings = SPISettings(1'000'000UL, MSBFIRST, SPI_MODE0);
        _spiMode = SPI_MODE0;
    } else if (_CS != -1) {
        // software SPI
        pinMode(_CLK, OUTPUT);
        pinMode(_CS, OUTPUT);
        pinMode(_MOSI, OUTPUT);
        pinMode(_MISO, INPUT);
    } else {
        _wire->begin();
        _i2caddr = i2caddr;
    }

    // try mode0
    if (getVersion() != 0x811) {
        if (_CS != -1 && _CLK == -1) {
            // Serial.println("try MODE1");
            _spisettings = SPISettings(1'000'000UL, MSBFIRST, SPI_MODE1);
            _spiMode = SPI_MODE1;

            if (getVersion() != 0x811) {
                return false;
            }
        } else {
            return false;
        }
    }
    writeRegister8(STMPE_SYS_CTRL1, STMPE_SYS_CTRL1_RESET);
    delay(10);

    for (uint8_t i = 0; i < 65; i++) {
        readRegister8(i);
    }

    writeRegister8(STMPE_SYS_CTRL2, 0x0); // turn on clocks!
    writeRegister8(STMPE_TSC_CTRL, STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN); // XYZ and enable!
    // Serial.println(readRegister8(STMPE_TSC_CTRL), HEX);
    writeRegister8(STMPE_INT_EN, STMPE_INT_EN_TOUCHDET);
    writeRegister8(STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT | (0x6 << 4)); // 96 clocks per conversion
    writeRegister8(STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ);
    writeRegister8(STMPE_TSC_CFG, STMPE_TSC_CFG_4SAMPLE | STMPE_TSC_CFG_DELAY_1MS | STMPE_TSC_CFG_SETTLE_5MS);
    writeRegister8(STMPE_TSC_FRACTION_Z, 0x6);
    writeRegister8(STMPE_FIFO_TH, 1);
    writeRegister8(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
    writeRegister8(STMPE_FIFO_STA, 0); // unreset
    writeRegister8(STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_50MA);
    writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
    writeRegister8(STMPE_INT_CTRL, STMPE_INT_CTRL_POL_HIGH | STMPE_INT_CTRL_ENABLE);

    return true;
}

/*!
 *  @brief  Returns true if touched, false otherwise
 *  @return True if if touched, false otherwise
 */
bool Adafruit_STMPE610::touched() const {
    return (readRegister8(STMPE_TSC_CTRL) & 0x80);
}

/*!
 *  @brief  Checks if buffer is empty
 *  @return True if empty, false otherwise
 */
bool Adafruit_STMPE610::bufferEmpty() const {
    return (readRegister8(STMPE_FIFO_STA) & STMPE_FIFO_STA_EMPTY);
}

/*!
 *  @brief  Returns the FIFO buffer size
 *  @return The FIFO buffer size
 */
uint8_t Adafruit_STMPE610::bufferSize() const {
    return readRegister8(STMPE_FIFO_SIZE);
}

/*!
 *  @brief  Returns the STMPE610 version number
 *  @return The STMPE610 version number
 */
uint16_t Adafruit_STMPE610::getVersion() const {
    // Serial.print("get version");
    uint16_t v = readRegister8(0);
    v <<= 8;
    v |= readRegister8(1);
    // Serial.print("Version: 0x"); Serial.println(v, HEX);
    return v;
}

/*!
 *  @brief  Reads touchscreen data
 *  @param  *x
 *	    The x coordinate
 *  @param  *y
 *	    The y coordinate
 *  @param  *z
 *	    The z coordinate
 */
void Adafruit_STMPE610::readData(uint16_t* x, uint16_t* y, uint8_t* z) const {
    uint8_t data[4];

    for (uint8_t i = 0; i < 4; i++) {
        data[i] = readRegister8(0xD7);
        // _spi->transfer(0x00);
        // Serial.print("0x");
        // Serial.print(data[i], HEX);
        // Serial.print(" / ");
    }
    *x = data[0];
    *x <<= 4;
    *x |= (data[1] >> 4);
    *y = data[1] & 0x0F;
    *y <<= 8;
    *y |= data[2];
    *z = data[3];
}

/*!
 *  @brief  Returns point for touchscreen data
 *  @return The touch point using TS_Point
 */
TS_Point Adafruit_STMPE610::getPoint() const {
    uint16_t x, y;
    uint8_t z;

    /* Making sure that we are reading all data before leaving */
    while (!bufferEmpty()) {
        readData(&x, &y, &z);
    }

    if (bufferEmpty()) {
        writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
    }

    return TS_Point(x, y, z);
}

/*!
 *  @brief  Reads 8bit of data from specified register
 *  @param  reg
 *          The register
 *  @return Data in the register
 */
uint8_t Adafruit_STMPE610::readRegister8(uint8_t reg) const {
    uint8_t x;
    if (_CS == -1) {
        // use i2c
        _wire->beginTransmission(_i2caddr);
        _wire->write((byte) reg);
        _wire->endTransmission();
        _wire->requestFrom(_i2caddr, (byte) 1);
        x = _wire->read();

        // Serial.print("$"); Serial.print(reg, HEX);
        // Serial.print(": 0x"); Serial.println(x, HEX);
    } else {
        if (_CLK == -1) {
            _spi->beginTransaction(_spisettings);
        }
        digitalWriteFast(_CS, LOW);
        delayMicroseconds(1); // CS_n falling to first capture clock: >= 1 us
        spiOut(0x80 | reg);
        spiOut(0x00);
        x = spiIn();
        delayMicroseconds(1); // Last clock edge to CS_n high: >= 1 us
        digitalWriteFast(_CS, HIGH);
        delayMicroseconds(1); // CS_n high to first clock edge: >= 300 ns
        if (_CLK == -1) {
            SPI.endTransaction();
        }
    }

    return x;
}

// /*!
//  *  @brief  Reads 16 bits of data from specified register
//  *  @param  reg
//  *          The register
//  *  @return Data in the register
//  */
// uint16_t Adafruit_STMPE610::readRegister16(uint8_t reg) const {
//     uint16_t x = 0;
//     if (_CS == -1) {
//         // use i2c
//         _wire->beginTransmission(_i2caddr);
//         _wire->write((byte) reg);
//         _wire->endTransmission();
//         _wire->requestFrom(_i2caddr, (byte) 2);
//         x = _wire->read();
//         x <<= 8;
//         x |= _wire->read();
//     }
//     if (_CLK == -1) {
//         // hardware SPI
//         if (_CLK == -1) {
//             _spi->beginTransaction(_spisettings);
//         }
//         digitalWriteFast(_CS, LOW);
//         delayMicroseconds(1); // CS_n falling to first capture clock: >= 1 us
//         spiOut(0x80 | reg);
//         spiOut(0x00);
//         x = spiIn();
//         x <<= 8;
//         x |= spiIn();
//         delayMicroseconds(1); // Last clock edge to CS_n high: >= 1 us
//         digitalWriteFast(_CS, HIGH);
//         delayMicroseconds(1); // CS_n high to first clock edge: >= 300 ns
//         if (_CLK == -1) {
//             _spi->endTransaction();
//         }
//     }
//
//     // Serial.print("$"); Serial.print(reg, HEX);
//     // Serial.print(": 0x"); Serial.println(x, HEX);
//     return x;
// }

/*!
 *  @brief  Writes 8 bit of data to specified register
 *  @param  reg
 *	    The register
 *  @param  val
 *          Value to write
 */
void Adafruit_STMPE610::writeRegister8(uint8_t reg, uint8_t val) const {
    if (_CS == -1) {
        // use i2c
        _wire->beginTransmission(_i2caddr);
        _wire->write((byte) reg);
        _wire->write(val);
        _wire->endTransmission();
    } else {
        if (_CLK == -1) {
            _spi->beginTransaction(_spisettings);
        }
        digitalWriteFast(_CS, LOW);
        delayMicroseconds(1); // CS_n falling to first capture clock: >= 1 us
        spiOut(reg);
        spiOut(val);
        delayMicroseconds(1); // Last clock edge to CS_n high: >= 1 us
        digitalWriteFast(_CS, HIGH);
        delayMicroseconds(1); // CS_n high to first clock edge: >= 300 ns
        if (_CLK == -1) {
            _spi->endTransaction();
        }
    }
}

/*!
 *  @brief  TS_Point constructor
 */
TS_Point::TS_Point() : x {}, y {}, z {} {}

/*!
 *  @brief  TS_Point constructor
 *  @param  x0
 *          Initial x
 *  @param  y0
 *          Initial y
 *  @param  z0
 *          Initial z
 */
TS_Point::TS_Point(int16_t x0, int16_t y0, int16_t z0) : x { x0 }, y { y0 }, z { z0 } {}

/*!
 *  @brief  Equality operator for TS_Point
 *  @return True if points are equal
 */
bool TS_Point::operator==(TS_Point p1) {
    return ((p1.x == x) && (p1.y == y) && (p1.z == z));
}

/*!
 *  @brief  Non-equality operator for TS_Point
 *  @return True if points are not equal
 */
bool TS_Point::operator!=(TS_Point p1) {
    return ((p1.x != x) || (p1.y != y) || (p1.z != z));
}
