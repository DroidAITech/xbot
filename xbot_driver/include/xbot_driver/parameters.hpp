/**
 * @file include/xbot_driver/parameters.hpp
 *
 * @brief Parameter configuration for the xbot.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef XBOT_PARAMETERS_HPP_
#define XBOT_PARAMETERS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief Parameter list and validator for the xbot.
 */
class Parameters
{
public:
  Parameters() :
    device_port("/dev/xbot"),
    sigslots_namespace("/xbot"),
    simulation(false),
    enable_acceleration_limiter(true),
    linear_acceleration_limit(0.3),
    linear_deceleration_limit(-0.3*1.2),
    angular_acceleration_limit(3.5),
    angular_deceleration_limit(-3.5*1.2)
  {
  } /**< @brief Default constructor. **/

  std::string device_port;         /**< @brief The serial device port name [/dev/xbot] **/
  std::string sigslots_namespace;  /**< @brief The first part of a sigslot connection namespace ["/xbot"] **/
  bool simulation;                 /**< @brief Whether to put the motors in loopback mode or not [false] **/
  bool enable_acceleration_limiter;/**< @brief Enable or disable the acceleration limiter [true] **/

  double linear_acceleration_limit;
  double linear_deceleration_limit;
  double angular_acceleration_limit;
  double angular_deceleration_limit;

  /**
   * @brief A validator to ensure the user has supplied correct/sensible parameter values.
   *
   * This validates the current parameters and if invalid, puts an error string in error_msg.
   *
   * @return bool : true if valid, false otherwise.
   */
  bool validate()
  {
    // not doing anything right now -  delete it, if we can find a use case ...
    return true;
  }

  std::string error_msg; /**< @brief Provides error messages when parameter validation fails (internal purposes only) **/
};

} // namespace xbot

#endif /* XBOT_PARAMETERS_HPP_ */
