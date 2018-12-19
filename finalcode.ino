/*
 * Simple Dynamixel MX sketch for Teensy/Arduino
 *
 * In this example we are going to set up a connection to a RS-485 based
 * Dynamixel bus on Serial1. The physical serial port Serial1 is on pins 0 and 1
 * on the Teensy 3.5. We will also set up a transmit enable pin on pin 2 to
 * enable transmitting with the RS-485 transceiver board. The Serial1 port will
 * need to be set to the appropriate baud rate of the servos; 57.6 kbaud is
 * the factory default, 115.2 kbaud and 1Mbaud is also often used.
 *
 * When the Dynamixel port is set up, the program continously pings a servo on
 * ID 1 and returns the result on the USB serial port (Serial).
 *
 * The ping function that is being used is very similar to other functions
 * defined for e.g. setting and getting the position of the servo. To
 * experiment, just substitute the ping with the functions definded above.
 *
 * A note on the functions. Basically, what the functions do is to call a set of
 * low-level functions in order to construct a proper formatted package to be
 * sent over the serial port to the Dynamixel using the Dynamixel protocol 2.0
 * (http://emanual.robotis.com/docs/en/dxl/protocol2/). Such functions are
 * sometimes called convenience functions. We have only implemented convenience
 * functions for a small number of often-used commands. If you need to send some
 * other command, you can implement the convenience function yourself, using the
 * existing convenience functions as templates and
 * (http://emanual.robotis.com/docs/en/dxl/mx/mx-64-2) as reference.
 *
 * If you are using an Arduino Mega, uncomment the #define ARDUINOMEMUSE line
 * below.
 */

/*
 * Copyright (c) 2018 Jens Dalsgaard Nielsen
 * Copyright (c) 2018 Karl Damkjær Hansen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include <math.h>
#include <elapsedMillis.h>

// Using Arduino Mega? Uncomment this:
//#define ARDUINOMEMUSE




#define HardwareSerial_h

#include <inttypes.h>

#include "Stream.h"


#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>
#include "Stream.h"
#include "uart.h"

enum SerialConfig {
    SERIAL_5N1 = UART_5N1,
    SERIAL_6N1 = UART_6N1,
    SERIAL_7N1 = UART_7N1,
    SERIAL_8N1 = UART_8N1,
    SERIAL_5N2 = UART_5N2,
    SERIAL_6N2 = UART_6N2,
    SERIAL_7N2 = UART_7N2,
    SERIAL_8N2 = UART_8N2,
    SERIAL_5E1 = UART_5E1,
    SERIAL_6E1 = UART_6E1,
    SERIAL_7E1 = UART_7E1,
    SERIAL_8E1 = UART_8E1,
    SERIAL_5E2 = UART_5E2,
    SERIAL_6E2 = UART_6E2,
    SERIAL_7E2 = UART_7E2,
    SERIAL_8E2 = UART_8E2,
    SERIAL_5O1 = UART_5O1,
    SERIAL_6O1 = UART_6O1,
    SERIAL_7O1 = UART_7O1,
    SERIAL_8O1 = UART_8O1,
    SERIAL_5O2 = UART_5O2,
    SERIAL_6O2 = UART_6O2,
    SERIAL_7O2 = UART_7O2,
    SERIAL_8O2 = UART_8O2,
};

enum SerialMode {
    SERIAL_FULL = UART_FULL,
    SERIAL_RX_ONLY = UART_RX_ONLY,
    SERIAL_TX_ONLY = UART_TX_ONLY
};

class HardwareSerial: public Stream
{
public:
    HardwareSerial(int uart_nr);
    virtual ~HardwareSerial() {}

    void begin(unsigned long baud)
    {
        begin(baud, SERIAL_8N1, SERIAL_FULL, 1);
    }
    void begin(unsigned long baud, SerialConfig config)
    {
        begin(baud, config, SERIAL_FULL, 1);
    }
    void begin(unsigned long baud, SerialConfig config, SerialMode mode)
    {
        begin(baud, config, mode, 1);
    }

    void begin(unsigned long baud, SerialConfig config, SerialMode mode, uint8_t tx_pin);

    void end();

    size_t setRxBufferSize(size_t size);

    void swap()
    {
        swap(1);
    }
    void swap(uint8_t tx_pin)    //toggle between use of GPIO13/GPIO15 or GPIO3/GPIO(1/2) as RX and TX
    {
        uart_swap(_uart, tx_pin);
    }

    /*
     * Toggle between use of GPIO1 and GPIO2 as TX on UART 0.
     * Note: UART 1 can't be used if GPIO2 is used with UART 0!
     */
    void set_tx(uint8_t tx_pin)
    {
        uart_set_tx(_uart, tx_pin);
    }

    /*
     * UART 0 possible options are (1, 3), (2, 3) or (15, 13)
     * UART 1 allows only TX on 2 if UART 0 is not (2, 3)
     */
    void pins(uint8_t tx, uint8_t rx)
    {
        uart_set_pins(_uart, tx, rx);
    }

    int available(void) override;

    int peek(void) override
    {
        // this may return -1, but that's okay
        return uart_peek_char(_uart);
    }
    int read(void) override
    {
        // this may return -1, but that's okay
        return uart_read_char(_uart);
    }
    int availableForWrite(void)
    {
        return static_cast<int>(uart_tx_free(_uart));
    }
    void flush(void) override;
    size_t write(uint8_t c) override
    {
        return uart_write_char(_uart, c);
    }
    inline size_t write(unsigned long n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(long n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(unsigned int n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(int n)
    {
        return write((uint8_t) n);
    }
    size_t write(const uint8_t *buffer, size_t size)
    {
        return uart_write(_uart, (const char*)buffer, size);
    }
    size_t write(const char *buffer)
    {
        return buffer? uart_write(_uart, buffer, strlen(buffer)): 0;
    }
    operator bool() const
    {
        return _uart != 0;
    }
    void setDebugOutput(bool);
    bool isTxEnabled(void)
    {
        return uart_tx_enabled(_uart);
    }
    bool isRxEnabled(void)
    {
        return uart_rx_enabled(_uart);
    }
    int baudRate(void)
    {
        return uart_get_baudrate(_uart);
    }

    bool hasOverrun(void)
    {
        return uart_has_overrun(_uart);
    }

    void startDetectBaudrate();

    unsigned long testBaudrate();

    unsigned long detectBaudrate(time_t timeoutMillis);

protected:
    int _uart_nr;
    uart_t* _uart = nullptr;
    size_t _rx_size;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;

#endif


// Buffers for transmitting (tx) and receiving (rx).
// NB: If you transmit or receive packages longer than 30 bytes, then change the
// length definitions below.
#define TX_BUFFER_LEN 30
#define RX_BUFFER_LEN 30
uint8_t tx_buffer[TX_BUFFER_LEN];
uint8_t rx_buffer[RX_BUFFER_LEN];


// Low-level functions for setting individual bytes in the buffers.
void putInt8t(int8_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = value;
}

int8_t getInt8t(uint8_t* buffer, size_t pos)
{
  return buffer[pos];
}

void putUint8t(uint8_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = value;
}

uint8_t getUint8t(uint8_t* buffer, size_t pos)
{
  return buffer[pos];
}

void putInt16t(int16_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = (uint8_t)(value & 0x00ff);
  buffer[pos + 1] = (uint8_t)(value >> 8);
}

void putUint16t(uint16_t value, uint8_t* buffer, size_t pos)
{
  buffer[pos] = (uint8_t)(value & 0x00ff);
  buffer[pos + 1] = (uint8_t)(value >> 8);
}

int16_t getInt16t(uint8_t* buffer, size_t pos)
{
  int16_t v = 0;
  v = buffer[pos + 1];
  v = v << 8;
  v = v | buffer[pos];
  return v;
}

uint16_t getUint16t(uint8_t* buffer, size_t pos)
{
  uint16_t v = 0;
  v = buffer[pos + 1];
  v = v << 8;
  v = v | buffer[pos];
  return v;
}

void putInt32t(int32_t val, uint8_t* buffer, size_t pos)
{
  for (int16_t i = 0; i < 4 ; i++) {
    buffer[pos + i] = (uint8_t)(val & 0x000000ff);
    val = val >> 8;
  }
}

int32_t getInt32t(uint8_t* buffer, size_t pos)
{
  int32_t v = 0;
  for (int16_t i = 3; i > -1 ; --i) {
    v = v << 8;
    v = v | (int32_t)buffer[pos + i];
  }
  return v;
}


// Dynamixel Protocol 2.0
// The protocol defines a header with fixed positions for the instruction and
// length fields:
#define DXL_LENGTH_POS 5
#define DXL_INSTRUCTION_POS 7

inline size_t getPackageLength(uint8_t* buffer)
{
  return getUint16t(buffer, DXL_LENGTH_POS);
}

// Cyclic Redundancy Check (CRC)
// The protocol uses CRC to check for faults in the transmissions, see:
// https://en.wikipedia.org/wiki/Cyclic_redundancy_check. Specifically
// Dynamixel uses a variant of the CRC16 algorithm used in MODBUS also called
// CRC-16-IBM, see http://emanual.robotis.com/docs/en/dxl/crc/.
#ifdef ARDUINOMEMUSE
const uint16_t  crc_table[] PROGMEM = {
#else
const uint16_t  crc_table[] = {
#endif

  0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
  0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
  0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
  0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
  0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
  0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
  0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
  0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
  0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
  0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
  0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
  0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
  0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
  0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
  0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
  0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
  0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
  0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
  0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
  0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
  0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
  0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
  0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
  0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
  0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
  0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
  0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};


uint16_t calculateCrc(uint16_t crc_accum, uint8_t* data_blk_ptr, size_t data_blk_size)
{
  size_t i, j;

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
#ifdef ARDUINOMEMUSE
    crc_accum = (crc_accum << 8) ^  pgm_read_word_near(crc_table + i);
#else
    crc_accum = (crc_accum << 8) ^ crc_table[i];
#endif
  }
  return crc_accum;
}

// Add CRC to a buffer
// Calculates the CRC and appends it to the buffer.
//
// @param buffer    The buffer to calculate CRC for.
// @param data_size The number of bytes in the buffer to calculate the CRC for.
//                  Incidentally, this is also the position in the array where
//                  the CRC will be added.
void addCrc(uint8_t* buffer, size_t data_size)
{
  uint16_t crc;
  crc = calculateCrc(0, buffer, data_size);
  putInt16t(crc, tx_buffer, data_size);
}

// Check the CRC of a package
// Calculates the CRC of the data in the buffer and compares it to the received
// CRC checksum.
//
// @param buffer the buffer holding the package.
// @param pos the position of the first CRC byte in the buffer.
// @return true if the CRC check is successful.
boolean checkCrc(uint8_t* buffer, int16_t pos)
{
  uint16_t incomming_crc = getUint16t(buffer, pos);
  uint16_t calculated_crc = calculateCrc(0, buffer, pos);
  return (calculated_crc == incomming_crc);
}

void setHdrAndID(uint8_t id)
{
  tx_buffer[0] = 0xff;
  tx_buffer[1] = 0xff;
  tx_buffer[2] = 0xfd;
  tx_buffer[3] = 0x00;
  tx_buffer[4] = id;
}


void dumpPackage(uint8_t *buffer)
{
  size_t l = getPackageLength(buffer) + 7;
  for (size_t i = 0; i < l; i++) {
    Serial.print((int)buffer[i], HEX); Serial.print(" ");
  }
  Serial.println("");
}

// Transmit the package in the tx_buffer
void transmitPackage()
{
  size_t pgk_length = getPackageLength(tx_buffer) + 7;
  Serial1.write(tx_buffer, pgk_length);
}

// Try to receive a package
//
// @param timeout milliseconds to wait for a reply.
// @returns true if a package was received and the CRC checks out.
bool receivePackage(size_t timeout = 100)
{
  elapsedMillis since_start = 0;
  size_t bytecount = 0;
  size_t remaining_read = 1;
  while (remaining_read > 0 && since_start < timeout)
  {
    if (Serial1.available())
    {
      uint8_t incomming_byte = Serial1.read();
      switch (bytecount)
      {
        case 0:
        case 1: if (incomming_byte == 0xFF) {
                  rx_buffer[bytecount] = incomming_byte;
                  ++bytecount;
                } else {
                  bytecount = 0;
                }
                break;
        case 2: if (incomming_byte == 0xFD)
                {
                  rx_buffer[bytecount] = incomming_byte;
                  ++bytecount;
                } else {
                  bytecount = 0;
                }
                break;
        case 3:
        case 4:
        case 5: rx_buffer[bytecount] = incomming_byte;
                ++bytecount;
                break;
        case 6: rx_buffer[bytecount] = incomming_byte;
                remaining_read = getPackageLength(rx_buffer);
                ++bytecount;
                break;
        default: rx_buffer[bytecount] = incomming_byte;
                 ++bytecount;
                 --remaining_read;
                 break;
      }
    }
  }
  if (remaining_read == 0)
  {
    //dumpPackage(rx_buffer, getPackageLength(rx_buffer));
    return checkCrc(rx_buffer, bytecount-2);
  }
  else
  {
    return false;
  }
}

// Send a READ instruction to the servo
void sendReadInstruction(uint8_t id, uint16_t from_addr, uint16_t data_length)
{
  /* Read instruction package
  0     1     2     3     4    5      6     7     8      9       10      11      12    13
  H1    H2    H3    RSRV  ID   LEN1   LEN2  INST  PARAM1 PARAM2  PARAM3  PARAM4  CRC1  CRC2
  0xFF  0xFF  0xFD  0x00  0x01  0x07  0x00  0x02  0x84   0x00    0x04    0x00    0x1D  0x15
  */
  setHdrAndID(id);
  putUint16t(7, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x02, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(from_addr, tx_buffer, 8);
  putUint16t(data_length, tx_buffer, 10);
  addCrc(tx_buffer, 12);

  transmitPackage();
}

// Send a 1 byte READ instruction to the servo
void sendWriteInstruction(uint8_t id, uint16_t address, uint8_t data)
{
  setHdrAndID(id);
  putUint16t(6, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x03, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(address, tx_buffer, 8);
  putUint8t(data, tx_buffer, 10);
  addCrc(tx_buffer, 11);

  transmitPackage();
}

// Send a 4 byte READ instruction to the servo
void sendWriteInstruction(uint8_t id, uint16_t address, int32_t data)
{
  /* Read instruction package
  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15
  H1   H2   H3   RSRV ID   LEN1 LEN2 INST P1   P2   P3   P4   P5   P6   CRC1 CRC2
  0xFF 0xFF 0xFD 0x00 0x01 0x09 0x00 0x03 0x74 0x00 0x00 0x02 0x00 0x00 0xCA 0x89
  */
  setHdrAndID(id);
  putUint16t(9, tx_buffer, DXL_LENGTH_POS);
  putUint8t(0x03, tx_buffer, DXL_INSTRUCTION_POS);
  putUint16t(address, tx_buffer, 8);
  putInt32t(data, tx_buffer, 10);
  addCrc(tx_buffer, 14);

  transmitPackage();
}


// Ping a servo
// Pings the servo and waits for a response. If none is received within the
// timeout, the ping fails. There is no check as to whether the response is
// appropriate. When sending a ping to broadcast (0xFE), several packages may be
// received. The user should look into the received package to see if the result
// is satisfactory.
//
// @param  id      the servo ID to ping.
// @param  timeout milliseconds before timing out.
// @return true if response is received.
bool doPing(uint8_t id, size_t timeout = 100)
{
  /* Ping Instruction Packet
    0     1     2     3     4     5     6     7     8     9
    H1    H2    H3    RSRV  ID    LEN1  LEN2  INST  CRC1  CRC2
    0xFF  0xFF  0xFD  0x00  0x01  0x03  0x00  0x01  0x19  0x4E
  */
  setHdrAndID(id);
  putUint16t(3, tx_buffer, DXL_LENGTH_POS); // length is at pos 5 and 6
  putUint8t(0x01, tx_buffer, DXL_INSTRUCTION_POS); // ping instruction (0x01) at pos 7
  addCrc(tx_buffer, 8); // CRC at pos 8 and 9
  transmitPackage();

  bool package_received = receivePackage(timeout);

  return package_received;
}

// Set the goal position of a servo
// Reads the position of a servo, if no response is read, the function does not
// modify the value of the position variable.
//
// @param id the servo ID to get the position of.
// @param position a reference to store the read position in.
// @return true if the servo responded.
bool readPosition(uint8_t id, int32_t& position)
{
  sendReadInstruction(id, 132, 4);
  if (receivePackage(100))
  {
    position = getInt32t(rx_buffer, 9);
    return true;
  }
  else
  {
    return false;
  }
}

// Read the position of a servo
//
// @param id       the servo ID to set the position of.
// @param position the position to go to.
// @return true if the servo responded.
bool setGoalPosition(uint8_t id, int32_t position)
{
  sendWriteInstruction(id, 116, position);
  if (receivePackage(100))
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Enable torque on the servo
// The torque must be enabled for the motor to move.
//
// @param id     the servo ID to set the position of.
// @param enable whether the torque should be enabled or not.
// @return true if the servo responded.
bool torqueEnable(uint8_t id, bool enable)
{
  uint8_t enable_data = 0x01;
  if (! enable)
  {
    enable_data = 0x00;
  }
  sendWriteInstruction(id, 64, enable_data);
  if (receivePackage(100))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void ping(){
  bool ping_success = doPing(1, 100);
 

 bool ping_successs = doPing(2, 100);
  

  bool ping_successss = doPing(3, 100);


  bool ping_successsss = doPing(4, 100);
  

  bool ping_successssss = doPing(5, 100);
}

void torque(int t1, int t2, int t3, int t4, int t5){
  if (t1 == 1){
  torqueEnable(1,true);
  } else {
    torqueEnable(1,false);
  }
   if (t2 == 1){
  torqueEnable(2,true);
  } else {
    torqueEnable(2,false);
  }
   if (t3 == 1){
  torqueEnable(3,true);
  } else {
    torqueEnable(3,false);
  }
   if (t4 == 1){
  torqueEnable(4,true);
  } else {
    torqueEnable(4,false);
  }
   if (t5 == 1){
  torqueEnable(5,true);
  } else {
    torqueEnable(5,false);
  }
}
int input_1;
int input_2;
int readinput_1;
int readinput_2;


float xr;
float yr;
float zr;
int servo4 = 1900;
int servo5 = 2100;
// These are the setup/loop functions that Arduino require.
void setup()
{
  // Initialize USB serial port. (On Teensy, the baud rate is disregarded and is
  // always 12 Mbaud).
  Serial.begin(9600);

  // Initialize the Dynamixel port (RX: pin0, TX: pin1, Transmit Enable: pin2.
  // With factory default baud rate of 57.6 kbaud).
  Serial1.begin(57600);
  Serial1.transmitterEnable(2);
  
pinMode(25,INPUT);
pinMode(26,INPUT);
pinMode(27,INPUT);
pinMode(28,INPUT);
pinMode(29,INPUT);
pinMode(30,INPUT);
pinMode(31,INPUT);
pinMode(32,INPUT);

torque(1,1,1,1,1);

int32_t initialposition1;
int32_t initialposition2;
int32_t initialposition3;

readPosition(1, initialposition1);
readPosition(2, initialposition2);
readPosition(3, initialposition3);

float initialtheta_1 = ((initialposition1-2000)/(4095/360));
float initialtheta_2 = (-1*((((initialposition2 +2960)/(4095/360)))- 540));
float initialtheta_3 = ((initialposition3- 2120)/(4095/360));

float goaltheta1=0;
float goaltheta2=45;
float goaltheta3=-90;

float deltatheta1 = goaltheta1 - initialtheta_1;
float deltatheta2 = goaltheta2 - initialtheta_2;
float deltatheta3 = goaltheta3 - initialtheta_3;

float t1=deltatheta1/70;
float t2=deltatheta2/70;
float t3=deltatheta3/70;

for (float j=1; j<71; j++){
initialtheta_1= initialtheta_1+ t1;
initialtheta_2= initialtheta_2+ t2;
initialtheta_3= initialtheta_3+ t3;

float real1 = 2040 +(4095/360)*initialtheta_1;
float real2 = 2960 -(4095/360)*initialtheta_2;
float real3 = 2120 +(4095/360)*initialtheta_3;

setGoalPosition(1,real1);
setGoalPosition(2,real2);
setGoalPosition(3,real3);


delay(0);

}

int32_t position1;
int32_t position2;
int32_t position3;

readPosition(1, position1);
readPosition(2, position2);
readPosition(3, position3);

float theta_1 = (((position1-2000)/(4095/360)))/57.2957795;
float theta_2 = (-1*((((position2 +2960)/(4095/360)))- 540))/57.2957795;
float theta_3 = ((position3- 2120)/(4095/360))/57.2957795;


xr = 110*cos(theta_1 + theta_2) + 110*cos(theta_1- theta_2) + 135*cos(theta_1 +theta_2 + theta_3 )+ 135*cos(theta_1 - theta_2 - theta_3);
yr = 110*sin(theta_1 + theta_2) + 110*sin(theta_1- theta_2) + 135*sin(theta_1 +theta_2 + theta_3 )+ 135*sin(theta_1 - theta_2 - theta_3);
zr = 220*sin( theta_2) + 270*sin( theta_2 + theta_3) + 56;

setGoalPosition(4,1800);
setGoalPosition(5,2200);

}
void loop()
{
  
ping();

float theta1;
float theta2;
float theta3;

float deltax= 0; 
float deltay= 0;
float deltaz= 0;
float velocity = 100;

//closegripper
if(digitalRead(26)==HIGH){
  int newangle4 = servo4 - 15;
  int newangle5 = servo5 + 15;
  if (1900>newangle4 && newangle4>1535 && 2100<newangle5 && newangle5<2558){
    servo4 = servo4 - 15;
    servo5 = servo5 + 15;
    setGoalPosition(4,servo4);
    setGoalPosition(5,servo5);
  }
}
//opengripper

if(digitalRead(25)==HIGH){
  int newangle4 = servo4 + 15;
  int newangle5 = servo5 - 15;
  if (1900>newangle4 && newangle4>1535 && 2100<newangle5 && newangle5<2558){
    servo4 = servo4 + 15;
    servo5 = servo5 - 15;
    setGoalPosition(4,servo4);
    setGoalPosition(5,servo5);
  }
}

if(digitalRead(32)==HIGH){
  deltaz=-5;
}
if(digitalRead(31)==HIGH){
  deltaz=5;
}

if(digitalRead(30)==HIGH){
  deltay=5;
}

if(digitalRead(29)==HIGH){
  deltay=-5;
}

if(digitalRead(28)==HIGH){
  deltax=-5;
}

if(digitalRead(27)==HIGH){
  deltax=5;
}

float projection_r;

float voltage4=0;
pinMode(30,INPUT);
voltage4 = digitalRead(30);
if (digitalRead(30)==1){

pinMode(30, OUTPUT);
 analogWrite(30, 0);
deltax=-6;
}

float voltage5=0;
pinMode(31,INPUT);
voltage5 = digitalRead(31);
if (digitalRead(31)==1){

pinMode(31, OUTPUT);
 analogWrite(31, 0);
deltay=6;
}

Serial.print("deltaz");
Serial.println(deltaz);
*/
float tra_length = sqrt(pow(deltax,2) + pow(deltay,2) + pow(deltaz,2));

int iterations= (tra_length*32)/velocity;


for(float i=0; i<=iterations; i++){

  readinput_2=1;
}


if(check4<2250&& check4>1750){

  readinput_2=0;
}


if(check4>2250){
  readinput_2=-1;
}

float xf = xr;
float yf = yr;
float zf = zr;

float t1 = deltax/iterations;
float t2 = deltay/iterations;
float t3 = deltaz/iterations;

xf += t1*2;
yf += t2*2;
zf += t3*2;

float r_f= sqrt(pow(xf,2) + pow(yf,2) + pow((zf-56),2));
float projection_r_f = sqrt( pow(xf,2) + pow(yf,2));
float theta1_f;
if (yf<0){
theta1 = -(acos(xf/projection_r_f))*57.2957795;
}else
{
theta1_f = (acos(xf/projection_r_f))*57.2957795;
}
float theta2_f = (atan((zf-56) / sqrt(pow(xf,2)+pow(yf,2))) + acos((pow(220,2)+pow(r_f,2)-pow(270,2))/(220*r_f*2) ))*57.2957795;
float theta3_f =((-3.145926 + acos((pow(220,2)+ pow(270,2) - pow(r_f,2)) /(220*270*2) )))*57.2957795;

if (zf>-170 && theta3_f<-1&& theta3_f> -110&& theta1_f<170 && theta1_f>-170 && projection_r_f>100/* && input_1 == readinput_1 && input_2 == readinput_2*/){
xr= xr+ t1;
yr= yr+ t2;
zr= zr+ t3;

float lk=  pow(xr,2) + pow(yr,2) + pow((zr-56),2);
float r= sqrt(lk);
projection_r = sqrt( pow(xr,2) + pow(yr,2));

if (yr<0){
theta1 = -(acos(xr/projection_r))*57.2957795;
}else
{  
theta1 = (acos(xr/projection_r))*57.2957795;
}     
theta2 = (atan((zr-56) / sqrt(pow(xr,2)+pow(yr,2))) + acos((pow(220,2)+pow(r,2)-pow(270,2))/(220*r*2) ))*57.2957795;
theta3 =((-3.145926 + acos((pow(220,2)+ pow(270,2) - pow(r,2)) /(220*270*2) )))*57.2957795;


if (zr>-170 && theta3<-1 && theta3> -110&& theta1<170 && theta1>-170 && projection_r>100 && input_1 == readinput_1 && input_2 == readinput_2){
float realposition1 = 2040 +(4095/360)*theta1;
float realposition2 = 2960 -(4095/360)*theta2;
float realposition3 = 2120 +(4095/360)*theta3;

  setGoalPosition(1,realposition1);
  setGoalPosition(2,realposition2);
  setGoalPosition(3,realposition3);
}
else{break;}
}else{break;}

}

}
