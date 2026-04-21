/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "operational_design_domain.hpp"
#include <csignal>
#include "adore_ros2_msgs/msg/odd.hpp"
#include "adore_ros2_msgs/msg/vehicle_info.hpp"
#include "adore_ros2_msgs/msg/weather.hpp"
#include <adore_dynamics_conversions.hpp>
#include <adore_map_conversions.hpp>
#include <ctime>

namespace adore
{

OperationalDeisngDomain::OperationalDeisngDomain( const rclcpp::NodeOptions& opts ) :
  rclcpp::Node{ "operational_design_domain", opts }
{
  load_parameters();
  setup_subscribers();
  setup_publishers();
}

void OperationalDeisngDomain::load_parameters()
{
  std::string openodd_file = declare_parameter<std::string>( "openodd_file", "" );

  if ( openodd_file == "" )
  {
    throw std::runtime_error( "Error when loading ODD file, no path was entered!");
  }

  // @TODO, add support for different openodd file types (yaml, xml?)
  load_json_odd_file(openodd_file);
}

void OperationalDeisngDomain::load_json_odd_file(const std::string& odd_file_path)
{
    std::ifstream ifs( odd_file_path );
    if( !ifs.is_open() )
    {
      throw std::runtime_error( "Could not open file: " + odd_file_path );
    }
    nlohmann::json j;
    ifs >> j;

    if ( j.contains("max_speed") )
    {
      odd.max_speed = j.at("max_speed").get<double>();
    }

    if ( j.contains("minimum_road_width") )
    {
      odd.minimum_road_width = j.at("minimum_road_width").get<double>();
    }

    if ( j.contains("valid_hours_of_the_day") )
    {
      odd.valid_hours_of_the_day = j.at("valid_hours_of_the_day").get<std::tuple<int, int>>();
    }

    if ( j.contains("max_wind_intensity") )
    {
      odd.max_wind_intensity = j.at("max_wind_intensity").get<double>();
    }
}

void OperationalDeisngDomain::setup_subscribers()
{
  timer = create_wall_timer( std::chrono::milliseconds( static_cast<int>( 100 ) ), // 10 Hz
                             std::bind( &OperationalDeisngDomain::timer_callback, this ) );

  subscriber_vehicle_state_dynamic = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>( "vehicle_state_dynamic", 1,
                                      [this](const adore_ros2_msgs::msg::VehicleStateDynamic& msg) {  latest_vehicle_state_dynamic = dynamics::conversions::to_cpp_type(msg); });

  subscriber_vehicle_info = create_subscription<adore_ros2_msgs::msg::VehicleInfo>( "vehicle_info", 1,
                                      [this](const adore_ros2_msgs::msg::VehicleInfo& msg) {  latest_vehicle_info = msg; });

  subscriber_local_map = create_subscription<adore_ros2_msgs::msg::Map>( "local_map", 1,
                                      [this](const adore_ros2_msgs::msg::Map& msg) {  latest_local_map = map::conversions::to_cpp_type(msg); });

  subscriber_weather = create_subscription<adore_ros2_msgs::msg::Weather>( "weather", 1,
                                      [this](const adore_ros2_msgs::msg::Weather& msg) { latest_weather = msg; });
}

void OperationalDeisngDomain::setup_publishers()
{
  publisher_odd = create_publisher<adore_ros2_msgs::msg::Odd>( "odd", 1 );
}

void OperationalDeisngDomain::timer_callback()
{
  std::optional<Violation> odd_violation = evaluate_odd();

  adore_ros2_msgs::msg::Odd odd_msgs;
  odd_msgs.time = now().seconds();
  odd_msgs.matching = !odd_violation.has_value();

  odd_msgs.status = "ODD/COD match";
  if ( odd_violation.has_value() )
  {
    odd_msgs.status = odd_violation.value();
  }

  publisher_odd->publish( odd_msgs );
}

std::optional<Violation> OperationalDeisngDomain::evaluate_odd()
{
  Violation violation = "";
  
  evaluate_max_speed(violation);
  evaluate_road_width(violation);
  evaluate_time(violation);
  evaluate_weather(violation);

  if ( violation.empty() )
  {
    return {};
  }

  return violation;
}

void OperationalDeisngDomain::evaluate_max_speed(Violation& violation)
{
  if ( !odd.max_speed.has_value() )
    return;

  if ( !latest_vehicle_info.has_value() )
  {
    violation += "vehicle's current or max speed could not be read, ";
    return;
  }

  if ( latest_vehicle_info.value().speed > odd.max_speed.value() )
  {
    violation += "vehicle's current speed is higher than defined odd max speed, ";
  }
  
  return;
}

void OperationalDeisngDomain::evaluate_road_width(Violation& violation)
{
  if ( !odd.minimum_road_width.has_value() )
    return;

  if ( !latest_local_map.has_value() )
  {
    violation += "vehicle's local map could not be read, ";
    return;
  }
  
  if ( !latest_vehicle_state_dynamic.has_value() )
  {
    violation += "vehicle's current position could not be read, ";
    return;
  }

  std::optional<double> lane_width = latest_local_map.value().get_nearest_lane_width( latest_vehicle_state_dynamic.value() );

  if ( !lane_width.has_value() )
  {
    violation += "unable to calculate lane width, ";
    return;
  }

  if ( lane_width.value() < odd.minimum_road_width.value() )
  {
    violation += "current road is not wide enough at " + std::to_string( lane_width.value() ) + "[m] of " + std::to_string( odd.minimum_road_width.value() ) + "[m], ";
    return;
  }

  return;
}

void OperationalDeisngDomain::evaluate_time(Violation& violation)
{
  if ( !odd.valid_hours_of_the_day.has_value() )
    return;

  int earlier_hour_of_the_day = std::get<0>(odd.valid_hours_of_the_day.value());
  int latest_hour_of_the_day = std::get<1>(odd.valid_hours_of_the_day.value());

  std::time_t current_time_seconds = now().seconds();
  // std::time_t time = static_cast<std::time_t>(current_time_seconds);

  std::tm* local = std::localtime(&current_time_seconds);
  int current_hour_of_the_day = local->tm_hour;
  int current_minnute_of_the_hour = local->tm_min;
  
  if ( current_hour_of_the_day < earlier_hour_of_the_day || current_hour_of_the_day > latest_hour_of_the_day )
  {
    violation += "vehicle is only allowed to drive between [" + std::to_string(earlier_hour_of_the_day) + ";" + std::to_string(latest_hour_of_the_day) + "] o'clock. Current time " + std::to_string(current_hour_of_the_day) + ":" + std::to_string(current_minnute_of_the_hour) + " o'clock, ";
    return;
  }

  return;
}

void OperationalDeisngDomain::evaluate_weather(Violation& violation)
{
  if ( !odd.max_wind_intensity.has_value() )
    return;

  if ( !latest_weather.has_value() )
  {
    violation += "weather could not be read, ";
    return;
  }

  if ( latest_weather.value().wind_intensity >= odd.max_wind_intensity.value() )
  {
    violation += "current weather wind is too intense at " + std::to_string( latest_weather.value().wind_intensity ) + "[m/s] of " + std::to_string( odd.max_wind_intensity.value() ) + "[m/s], ";
    return;
  }

  return;
}

} // namespace adore
