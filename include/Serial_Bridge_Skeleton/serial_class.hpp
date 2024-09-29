#include <cstddef>
#include <cstdint>
#include <cstring>
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <string>
#include <iostream>

#include <chrono>
#include <thread>

#define START_BYTE 0xAA
#define END_BYTE 0xBB

#pragma pack(push, 1)
struct CommunicationData {
  uint8_t start_byte;

  // Twist For Base
  uint8_t x;
  uint8_t y;
  uint8_t omega;

  // Vector For Shooter
  uint8_t ax;
  uint8_t ay;
  uint8_t az;

  // Position of Catcher
  uint8_t on_off;
  uint8_t theta;
  uint8_t zz;

  // end byte and crc
  uint8_t end_byte;
  uint8_t crc;
};

#pragma pack(pop)

void print_this(CommunicationData cd) {
  std::cout << "####COMMUNICATIONDATA####" << std::endl
            << "x:\t" << (int)cd.x << std::endl
            << "y:\t" << (int)cd.y << std::endl
            << "omega:\t" << (int)cd.omega << std::endl
            << "ax:\t" << (int)cd.ax << std::endl
            << "ay:\t" << (int)cd.ay << std::endl
            << "az:\t" << (int)cd.az << std::endl
            << "on_off:\t" << (int)cd.on_off << std::endl
            << "theta:\t" << (int)cd.theta << std::endl
            << "zz:\t" << (int)cd.zz << std::endl;


  //  std::cout << "DATA IN" << std::endl;
}

enum State {
  START_BYTE_FOUND,
  START_BYTE_NOT_FOUND,
  
};



const uint8_t crc8x_table[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA,
    0x7D, 0x4C, 0x1F, 0x2E, 0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D, 0x86, 0xB7, 0xE4, 0xD5,
    0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F,
    0xB8, 0x89, 0xDA, 0xEB, 0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13, 0x7E, 0x4F, 0x1C, 0x2D,
    0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51,
    0xC6, 0xF7, 0xA4, 0x95, 0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6, 0x7A, 0x4B, 0x18, 0x29,
    0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3,
    0x44, 0x75, 0x26, 0x17, 0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2, 0xBF, 0x8E, 0xDD, 0xEC,
    0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD,
    0x3A, 0x0B, 0x58, 0x69, 0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A, 0xC1, 0xF0, 0xA3, 0x92,
    0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68,
    0xFF, 0xCE, 0x9D, 0xAC};

uint8_t calculate_cr8x_fast(uint8_t *data, size_t len) {
  uint8_t crc = 0xFF; // init value
  for (size_t i = 0; i < len; i++) {
    crc = crc8x_table[data[i] ^ crc];
  }
  return crc;
}


bool verify_communication(CommunicationData d) {

  if (d.start_byte != START_BYTE) return false;

  if (d.end_byte != END_BYTE) return false;

  uint8_t current_crc = calculate_cr8x_fast((uint8_t*)(&d), sizeof(d) - sizeof(d.crc));

  if (d.crc != current_crc) return false;

  return true;
  
}

void make_sendable_with_metadata(CommunicationData & c) {
  c.start_byte = START_BYTE;
  c.end_byte = END_BYTE;
  c.crc = calculate_cr8x_fast((uint8_t*)&c, sizeof(c) - sizeof(c.crc));
}




class BridgeClass {
  public:
  BridgeClass(std::string port): Bridge(port) {
    Bridge.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    
    Bridge.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    
    Bridge.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    
    Bridge.SetParity(LibSerial::Parity::PARITY_NONE);

    current_state = START_BYTE_NOT_FOUND;

  }

  ~BridgeClass() {
    Bridge.Close();
  }


  CommunicationData attempt_get_non_blocking() {
    // Clear The Buffer
    memset(&Buffer, 0, sizeof(Buffer));
    byte_buffer = 0x00;
    
    switch (current_state) {
    case START_BYTE_FOUND:
      current_state = START_BYTE_NOT_FOUND;

      Buffer.start_byte = START_BYTE;
      DataBufferFromLib.resize(sizeof(Buffer) - sizeof(Buffer.start_byte));

      try {
	
	Bridge.Read(DataBufferFromLib, sizeof(Buffer) - sizeof(Buffer.start_byte), 10); // buffer , size , timeout_ms
      } catch (...) {
	std::cout << "[ERROR] Exceded Time limit 10ms to read Buffer " << std::endl;
      }


      if (DataBufferFromLib.size() != sizeof(Buffer) - sizeof(Buffer.start_byte))
	std::cout << "[URGENT] BUFFER SIZE DOES NOT MATCH" << std::endl;
											   

      std::memcpy(&(Buffer.x), DataBufferFromLib.data(), sizeof(Buffer) - sizeof(Buffer.start_byte)); //
      DataBufferFromLib.clear(); // Clean the buffer

      if (!verify_communication(Buffer)) throw -2;
      else return Buffer;
      break;

    case START_BYTE_NOT_FOUND:

      try {
	Bridge.ReadByte(byte_buffer);
      } catch (...) {
	std::cout << "[ERROR] Error Occoured While Reading Byte" << std::endl;
      }

      if (byte_buffer == START_BYTE) current_state = START_BYTE_FOUND;
      throw -1;
      break;
    }
    
    // You Will Never Reach Here;
    return Buffer;
  }

  bool attempt_send_probably_blocking(CommunicationData d) {
    if (!verify_communication(d)) return false;
    // Clear The Data Buffer Just To Be Sure
    DataBufferFromLib.clear();
    DataBufferFromLib.resize(sizeof(d));
    memcpy(DataBufferFromLib.data(), &d, sizeof(d)); // store d in the buffer

    try {
      Bridge.Write(DataBufferFromLib);
    } catch (...) {
      throw -1;
    }

    return true;
  }

  void drian_attempt_blocking() {
    Bridge.DrainWriteBuffer();
  }
  
  private:


  LibSerial::DataBuffer DataBufferFromLib;

  CommunicationData Buffer;
  
  LibSerial::SerialPort Bridge;

  enum State current_state;

  uint8_t byte_buffer;
  
};

