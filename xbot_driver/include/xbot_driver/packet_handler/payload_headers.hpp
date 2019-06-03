/**
 * @file include/xbot_driver/packet_handler/payload_headers.hpp
 *
 * @brief Byte id's for the individual payload headers.
 *
 * Each part of a xbot packet carries one or more payload chunks. Each chunk
 * is id'd by one of the values here.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef XBOT_PAYLOAD_HEADERS_HPP_
#define XBOT_PAYLOAD_HEADERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

class Header {
public:
  enum PayloadType {
  // Streamed payloads
  CoreSensors = 16, ImuSensors = 17
  };
};

} // namespace xbot

#endif /* XBOT_PAYLOAD_HEADERS_HPP_ */
