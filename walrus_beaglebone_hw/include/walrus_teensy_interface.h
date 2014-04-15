#ifndef WALRUS_TEENSY_INTERFACE_H_
#define WALRUS_TEENSY_INTERFACE_H_

#include <device_driver_base/serial_port.h>
#include <boost/shared_ptr.hpp>
#include <stdint.h>

using namespace device_driver;

#define TEENSY_PING_COMMAND_ID (0x01)
#define TEENSY_ENABLE_COMMAND_ID (0x02)
#define TEENSY_DISABLE_COMMAND_ID (0x03)
#define TEENSY_SET_PWM_COMMAND_ID (0x04)
#define TEENSY_GET_ANALOG_COMMAND_ID (0x05)
#define TEENSY_GET_VELOCITY_COMMAND_ID (0x06)
#define TEENSY_GET_POSITION_COMMAND_ID (0x07)
static const uint8_t TEENSY_START_BYTE = 0x5E;

class WalrusTeensyInterface
{
 private:
  DriverSerialPortPtr serial_port;
  unsigned int timeout;
 public:
 WalrusTeensyInterface(unsigned int timeout = 10):
  serial_port(new DriverSerialPort()), timeout(timeout){
  }
  void open(){
    serial_port->open("/dev/ttyACM0", 115200, 8, serial_parity_none);
    /*try{
      int result = ping();
      if(result != 0){
        ROS_ERROR("Error reading pingback from teensy");
        serial_port->close();
      }
    } catch(...){
      ROS_ERROR("Could not ping teensy");
      serial_port->close();
      }*/
  }
  
  void send_command(uint8_t command, uint8_t data_length, uint8_t* data){
    serial_port->write(&TEENSY_START_BYTE, 1, timeout);
    uint8_t length = data_length + 1 + 1;
    serial_port->write(&length, 1, timeout);
    serial_port->write(&command, 1, timeout);
    serial_port->write(data, data_length, timeout);
    uint8_t sum = TEENSY_START_BYTE+command+length;
    for(int i = 0; i<data_length; ++i)
      sum += data[i];
    uint8_t checksum = 0xFF - sum;
    serial_port->write(&checksum, 1, timeout);
  }
  int receive_result(uint8_t buffer_size, uint8_t* buffer){
    uint8_t message_beginning[2];
    serial_port->read_from_header(&TEENSY_START_BYTE, 1, message_beginning, 2, timeout);
    uint8_t length = message_beginning[1];
    uint8_t data_length = length - 1;
    if(buffer_size < data_length){
      ROS_WARN("Receive buffer not big enough... was %d, needed %d", buffer_size, data_length);
      return -1;
    }
    serial_port->read(buffer, data_length, timeout);
    uint8_t checksum;
    serial_port->read(&checksum, 1, timeout);

    uint8_t sum = TEENSY_START_BYTE+length+checksum;
    for(int i = 0; i<data_length; ++i)
      sum += buffer[i];
    if(sum == 0xFF)
      return data_length;
    return -1;
  }

  int send_command_with_result(uint8_t command, uint8_t data_length, uint8_t* data, uint8_t receive_buffer_size, uint8_t* receive_buffer){
    send_command(command, data_length, data);
    return receive_result(receive_buffer_size, receive_buffer);
  }

  int ping(){
    return send_command_with_result(TEENSY_PING_COMMAND_ID, 0, NULL, 0, NULL);
  }
  int enable(){
    return send_command_with_result(TEENSY_ENABLE_COMMAND_ID, 0, NULL, 0, NULL);
  }
  int disable(){
    return send_command_with_result(TEENSY_DISABLE_COMMAND_ID, 0, NULL, 0, NULL);
  }
  int set_pwm(uint8_t channel, uint16_t period_us){
    uint8_t data[3];
    data[0] = channel;
    data[1] = ((uint8_t*)&period_us)[0];
    data[2] = ((uint8_t*)&period_us)[1];
    return send_command_with_result(TEENSY_SET_PWM_COMMAND_ID, 3, data, 0, NULL);
  }
  template<class value_type> value_type get_value_from_channel(uint8_t command, uint8_t channel){
    value_type result;
    int num_read = send_command_with_result(command, 1, &channel, sizeof(result), (uint8_t*)&result);
    if(num_read == sizeof(result))
      return result;
    return -1;
  }
  int16_t get_analog(uint8_t channel){
    return get_value_from_channel<int16_t>(TEENSY_GET_ANALOG_COMMAND_ID, channel);
  }
  int16_t get_velocity(uint8_t channel){
    return get_value_from_channel<int16_t>(TEENSY_GET_VELOCITY_COMMAND_ID, channel);
  }
  int32_t get_position(uint8_t channel){
    return get_value_from_channel<int32_t>(TEENSY_GET_POSITION_COMMAND_ID, channel);
  }


};
typedef boost::shared_ptr<WalrusTeensyInterface> WalrusTeensyInterfacePtr;

#endif
