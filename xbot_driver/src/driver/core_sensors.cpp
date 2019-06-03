/**
 * @file /xbot_driver/src/driver/core_sensors.cpp
 *
 * @brief Implementation of the core sensor packet data.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/packets/core_sensors.hpp"
#include "../../include/xbot_driver/packet_handler/payload_headers.hpp"
#include <time.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool CoreSensors::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
//  buildBytes(Header::CoreSensors, byteStream);
//  buildBytes(length, byteStream);
//  buildBytes(data.time_stamp, byteStream);	//2
//  buildBytes(data.bumper, byteStream);		//1
//  buildBytes(data.wheel_drop, byteStream);	//1
//  buildBytes(data.cliff, byteStream);		//1
//  buildBytes(data.left_encoder, byteStream);	//2
//  buildBytes(data.right_encoder, byteStream);	//2
//  buildBytes(data.left_pwm, byteStream);	//1
//  buildBytes(data.right_pwm, byteStream);	//1
//  buildBytes(data.buttons, byteStream);		//1
//  buildBytes(data.charger, byteStream);		//1
//  buildBytes(data.battery, byteStream);		//1
//  buildBytes(data.over_current, byteStream);	//1

  return true;
}



bool CoreSensors::deserialise(ecl::PushAndPop<unsigned char> & byteStream)
{
//    std::cout<<byteStream.size()<<std::endl;

//  if (byteStream.size() < length+2)
//  {
//    std::cout<<"length:"<<(unsigned int)length<<std::endl;
//    std::cout << "bytestream.size:"<<byteStream.size()<<std::endl<<"xbot_node: xbot_default: deserialise failed. not enough byte stream." << std::endl;
//    return false;
//  }
  unsigned char data_type;//0x10 marker
  buildVariable(data_type, byteStream);

//  std::cout<<"header_id:"<<(unsigned int)header_id<<std::endl;
  unsigned char power_num;//0x01 num of power
  buildVariable(power_num, byteStream);
//  std::cout<<"power_num:"<<(unsigned int)power_num<<std::endl;
  uint16_t tempvariable = 0;

  buildVariable(tempvariable,byteStream);
  data.battery_voltage = tempvariable/100.0;
//  std::cout<<"power_voltage:"<<data.power_voltage<<std::endl;
  unsigned char infred_num;//0x08
  buildVariable(infred_num, byteStream);
//clockwise direction, the left or right is based on the robot face
  buildVariable(tempvariable,byteStream);
  queue_rear_left_infrared.lpush(tempvariable);
  data.rear_left_infred = queue_rear_left_infrared.mean();

  buildVariable(tempvariable,byteStream);
  queue_rear_center_infrared.lpush(tempvariable);
  data.rear_center_infred = queue_rear_center_infrared.mean();

  buildVariable(tempvariable,byteStream);
  queue_rear_right_infrared.lpush(tempvariable);
  data.rear_right_infred = queue_rear_right_infrared.mean();

  buildVariable(tempvariable,byteStream);
  queue_front_left_infrared.lpush(tempvariable);
  data.front_left_infred = queue_front_left_infrared.mean();

  buildVariable(tempvariable,byteStream);
  queue_front_center_infrared.lpush(tempvariable);
  data.front_center_infred = queue_front_center_infrared.mean();

  buildVariable(tempvariable,byteStream);
  queue_front_right_infrared.lpush(tempvariable);
  data.front_right_infred = queue_front_right_infrared.mean();

  buildVariable(tempvariable,byteStream);
  queue_dock_left_infrared.lpush(tempvariable);
  data.dock_left_infred = queue_dock_left_infrared.mean();

  buildVariable(tempvariable,byteStream);
  queue_dock_left_infrared.lpush(tempvariable);
  data.dock_left_infred = queue_dock_left_infrared.mean();
//  std::cout<<"rear_center_infred:"<<data.rear_center_infred<<std::endl;

  unsigned char current_num;//0x05
  buildVariable(current_num,byteStream);

  buildVariable(tempvariable,byteStream);
  data.front_left_current = tempvariable/100.0;

  buildVariable(tempvariable,byteStream);
  data.front_right_current = tempvariable/100.0;

  buildVariable(tempvariable,byteStream);
  data.rear_left_current = tempvariable/100.0;

  buildVariable(tempvariable,byteStream);
  data.rear_right_current = tempvariable/100.0;

  buildVariable(tempvariable,byteStream);
  data.up_down_current = tempvariable/100.0;

  unsigned char echo_num;//0x06
  buildVariable(echo_num,byteStream);
//clockwise direction, the left or right is based on the robot face

//  rear right
  buildVariable(tempvariable,byteStream);

  queue_rear_right_echo.lpush(tempvariable);

  data.rear_right_echo = queue_rear_right_echo.mean();

//  rear center
  buildVariable(tempvariable,byteStream);

  queue_rear_center_echo.lpush(tempvariable);

  data.rear_center_echo = queue_rear_center_echo.mean();

//  rear left
  buildVariable(tempvariable,byteStream);

  queue_rear_left_echo.lpush(tempvariable);

  data.rear_left_echo = queue_rear_left_echo.mean();

//  front left
  buildVariable(tempvariable,byteStream);

  queue_front_left_echo.lpush(tempvariable);

  data.front_left_echo = queue_front_left_echo.mean();


//  front center
  buildVariable(tempvariable,byteStream);

  queue_front_center_echo.lpush(tempvariable);

  data.front_center_echo = queue_front_center_echo.mean();

//  front right
  buildVariable(tempvariable,byteStream);

  queue_front_right_echo.lpush(tempvariable);

  data.front_right_echo = queue_front_right_echo.mean();

//  std::cout<<"front_center_echo:"<<data.front_center_echo<<std::endl;
  unsigned char encoder_num;
  buildVariable(encoder_num, byteStream);
  buildVariable(data.front_left_encoder, byteStream);
//  std::cout<<"front_left:"<<data.front_left_encoder<<std::endl;
  buildVariable(data.front_right_encoder, byteStream);
//  std::cout<<"front_right:"<<data.front_right_encoder<<std::endl;
  buildVariable(data.rear_left_encoder, byteStream);
  buildVariable(data.rear_right_encoder, byteStream);
  buildVariable(data.up_down_encoder, byteStream);

  unsigned char imu_num;
  buildVariable(imu_num, byteStream);
  buildVariable(data.acce_x, byteStream);
  buildVariable(data.acce_y, byteStream);
  buildVariable(data.acce_z, byteStream);
  buildVariable(data.gyro_x, byteStream);
  buildVariable(data.gyro_y, byteStream);
  buildVariable(data.gyro_z, byteStream);
  buildVariable(data.mag_x, byteStream);
  buildVariable(data.mag_y, byteStream);
  buildVariable(data.mag_z, byteStream);
  buildVariable(data.pressure, byteStream);
  buildVariable(data.yaw,byteStream);
  buildVariable(data.pitch, byteStream);
  buildVariable(data.roll, byteStream);
  buildVariable(data.timestamp, byteStream);

  std::cout<<"timestamp:"<<data.timestamp<<std::endl;
//  unsigned short x=3;
//  unsigned short y=65500;
//  unsigned short result=x-y;

//  std::cout<<"sizeof timestamp:"<<result<<std::endl;


//  std::cout<<"time:"<<time(0)<<"|left_encoder:"<<data.left_encoder<<std::endl;


//  std::cout<<"power:"<<data.power_voltage<<"|Echo1:"<<data.echo_1<<"|Echo2:"<<data.echo_2<<"|Echo3:"<<data.echo_3<<"|Echo4:"<<data.echo_4<<std::endl;
  return true;
}

void CoreSensors::build_special_variable(float &variable, ecl::PushAndPop<unsigned char> &byteStream)
{
    if (byteStream.size() < 2)
      return;

    unsigned char a, b;
    buildVariable(a,byteStream);
    buildVariable(b,byteStream);
    variable = ((unsigned int)(a&0x0f))/100.0;

    variable += ((unsigned int)(a>>4))/10.0;

    variable += (unsigned int)(b&0x0f);

    variable += ((unsigned int)(b>>4))*10;


}


} // namespace xbot
