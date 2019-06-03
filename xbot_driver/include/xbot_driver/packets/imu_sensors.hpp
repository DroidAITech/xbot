/**
 * @file include/xbot_driver/packets/core_sensors.hpp
 *
 * @brief Imu sensor packet payloads.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 */
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#ifndef XBOT_IMU_SENSORS_HPP__
#define XBOT_IMU_SENSORS_HPP__

/*****************************************************************************
** Include
*****************************************************************************/

#include "../packet_handler/payload_base.hpp"
#include "../macros.hpp"
#include <stdint.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot
{

/*****************************************************************************
** Interface
*****************************************************************************/

class xbot_PUBLIC ImuSensors : public packet_handler::payloadBase
{
public:
  ImuSensors() : packet_handler::payloadBase(false, 32) {};

  struct Data {
    short acce_x;
    short acce_y;
    short acce_z;
    short gyro_x;
    short gyro_y;
    short gyro_z;
    short mag_x;
    short mag_y;
    short mag_z;
    float pressure;
    short yaw;
    short pitch;
    short roll;
    unsigned int timestamp;

  } data;


  bool serialise(ecl::PushAndPop<unsigned char> & byteStream);
  bool deserialise(ecl::PushAndPop<unsigned char> & byteStream);
};

} // namespace xbot

#endif /* XBOT_IMU_SENSORS_HPP__ */
