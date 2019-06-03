/**
 * @file /xbot_driver/src/driver/imu_sensors.cpp
 *
 * @brief Implementation of the imu sensor packet data.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_imu/hydro-devel/xbot_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/packets/imu_sensors.hpp"
#include "../../include/xbot_driver/packet_handler/payload_headers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool ImuSensors::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
//  buildBytes(Header::ImuSensors, byteStream);
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
bool ImuSensors::deserialise(ecl::PushAndPop<unsigned char> & byteStream)
{
//  if (byteStream.size() < length+2)
//  {
//      std::cout<<"length:"<<(unsigned int)length<<std::endl;
//    std::cout << "bytestream.size:"<<byteStream.size()<<std::endl<<"xbot_node: xbot_default: deserialise failed. not enough byte stream." << std::endl;
//    return false;
//  }

  unsigned char header_id;
  buildVariable(header_id, byteStream);

  if( header_id != Header::ImuSensors )
  {
      std::cout<<"header_id is wrong. header_id:"<<(unsigned int)header_id<<std::endl;

      return false;
}
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
//  buildVariable(data.yaw, byteStream);
  std::cout<<"yaw: "<<data.yaw<<std::endl;
  buildVariable(data.pitch, byteStream);
  buildVariable(data.roll, byteStream);

  buildVariable(data.timestamp, byteStream);





//  std::cout<<"sucessed!"<<std::endl;

  return true;
}



} // namespace xbot
