#include <Arduino.h>
#include <HardwareSerial.h>

#define DEBUG true

// Modbus commands
#define READ_HOLDING_REGISTER   (byte) 0x03
#define WRITE_SINGLE_COIL       (byte) 0x05
#define WRITE_SINGLE_REGISTER   (byte) 0x06
#define WRITE_MULTIPLE_REGISTER (byte) 0x10

// NP785-05 parameters
#define REQUEST_TIMEOUT    100 // time in ms
#define RESPONSE_TIMEOUT   100 // time in ms
#define DEFAULT_BAUD       19200
#define BROADCAST_ADDRESS  0x00
#define SIGNOW_SET_ADDRESS 0x03
#define REQ_PKT_LENGTH 8
#define RSP_PKT_LENGTH 7

// Board specific parameters
#define TX_PIN 17
#define RX_PIN 18

// Function declarations
void create_modbus_request(
  byte* frame,
  byte device_address,
  byte modbus_command,
  byte address_high,
  byte address_low,
  byte rw_high,
  byte rw_low);
void send_modbus_request(byte* frame, byte length);   // implemented
bool _send_modbus_request(byte* payload, byte* recv_payload, uint8_t pl_sz, uint8_t recvpl_sz);
bool read_modbus_response(byte* frame, byte length);  // implemented
bool verify_CRC(byte* frame, byte length);            // implemented

uint16_t calculate_CRC(byte* frame, byte length);     // implemented
uint8_t find_device_address(HardwareSerial* device, uint8_t address_address);

void flush_serial_input(HardwareSerial* device);

// Globals
HardwareSerial NP785(1);

/* 
  - Master Write Request Packet format 
  Byte IDX | Name
     0     | Slave Address
     1     | Function code
     2     | Register Address Hi
     3     | Register Address Lo
     4     | Data Hi
     5     | Data Lo
     6     | CRC  Lo
     7     | CRC  Hi


     7 6 5 4 3 2 1 

  - Master Read Request Packet format 
  Byte IDX | Name
     0     | Slave Address
     1     | Function code
     2     | Register Address Hi
     3     | Register Address Lo
     4     | Num Registers to Read Hi
     5     | Num Registers to Read Lo
     6     | CRC  Lo
     7     | CRC  Hi

  - Slave Response Packet format
  Byte IDX | Name
     0     | Slave Address
     1     | Function code
     2     | Byte Count (2)
     3     | Data Hi
     4     | Data Low
     5     | CRC  Lo
     6     | CRC  Hi

*/

void setup(){
  Serial.begin(115200); // My terminal serial comm port
  NP785.begin(DEFAULT_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(2000); // Initialization delay

  Serial.println("BEGINNING TRANSMISSION NP785 test module");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  find_device_address(&NP785, 139);
}

void create_modbus_request(
  byte* frame,
  byte device_address,
  byte modbus_command,
  byte address_high,
  byte address_low,
  byte rw_high, // For a read request, register address, for a write request, Data
  byte rw_low){
    frame[0] = device_address;
    frame[1] = modbus_command;
    frame[2] = address_high;
    frame[3] = address_low;
    frame[4] = rw_high;
    frame[5] = rw_low;

    uint16_t crc = calculate_CRC(frame, 6);
    frame[6] = crc & 0xFF;          // CRC Lo
    frame[7] = (crc >> 8) & 0xFF;   // CRC Hi
  }

void send_modbus_request(byte* frame, byte length){
  for(uint8_t i = 0; i < length; i++){
    NP785.write(frame[i]);
  }
}

bool read_modbus_response(byte* frame, byte length){
  uint8_t bytes_read = 0;
  unsigned long start_time = millis();

  while(bytes_read < length && (millis() - start_time) < RESPONSE_TIMEOUT){
    if(NP785.available()){
      frame[bytes_read] = NP785.read();
      bytes_read++;
    }
  }

  if(bytes_read == length){
    if(DEBUG)
      Serial.printf("Read %d bytes\n", bytes_read);
    return true;
  } else{
    if(DEBUG)
      Serial.printf("Response timeout %d ms, Bytes read %d out of %d.\n", RESPONSE_TIMEOUT, bytes_read, length);
    return false;
  }
}

uint16_t calculate_CRC(byte* frame, byte length){
  uint16_t CRC = 0xFFFF;
  uint16_t FCS = 0xA001;
  
  for(byte i = 0; i < length; i++){
    CRC ^= frame[i]; // XOR the frame byte with the CRC
    for(byte j = 0; j < 8; j++){
      if(CRC & 0x0001){ // Check if LSB of CRC is 1 (we got a divisor)
        CRC >>=1;    // Right shift CRC by 1
        CRC ^= FCS;  // XOR with 'Polynomial' (need to read up on CRC steps)
      }else{
        CRC >>=1;    // Right shift CRC by 1
      }
    }
  }
  return CRC;
}

bool verify_CRC(byte* frame, byte length){
  // Does not verify the request CRC is a valid CRC for the response packet
  uint16_t received_CRC = (frame[length-1] << 8) | (frame[length-2]); // CRC Hi and CRC Lo respectively (See packet format)
  return received_CRC == calculate_CRC(frame, length-2);
}

void flush_serial_input(HardwareSerial* device){
  while(device->available()){
    device->read();
  }
}

bool _send_modbus_request(byte* payload, byte* recv_payload, uint8_t pl_sz, uint8_t recvpl_sz){
  send_modbus_request(payload, pl_sz);
  return read_modbus_response(recv_payload, recvpl_sz);
}

uint8_t find_device_address(HardwareSerial* device, uint8_t address_address){
  byte request_frame[REQ_PKT_LENGTH];
  byte response_frame[RSP_PKT_LENGTH];
  
  Serial.println("Begin looking for address:");
  for(uint8_t x = 1; x < 255; x++){

    create_modbus_request(
      request_frame,
      x,
      READ_HOLDING_REGISTER,
      0x00,            // address hi
      address_address, // address lo
      0x00,            // number of register to read hi
      0x01             // number of register to read lo
    );

    flush_serial_input(device);
    bool rcvd = _send_modbus_request(request_frame, response_frame, REQ_PKT_LENGTH, RSP_PKT_LENGTH);
    if(!rcvd){
      Serial.printf("[Timeout] 0x%x\n", x);
    }else if(verify_CRC(response_frame, RSP_PKT_LENGTH)){
        Serial.print("[0x] ");
        for(int i = RSP_PKT_LENGTH-1; i >= 0; i--){
          Serial.printf("%x ", response_frame[i]);
        }
        memset(response_frame, 0, RSP_PKT_LENGTH); // Reset buffer to prevent false positives
        
        Serial.println();
        Serial.printf("ADDRESS FOUND: 0x%x\n", x);
    }
  }
  return 0;
}