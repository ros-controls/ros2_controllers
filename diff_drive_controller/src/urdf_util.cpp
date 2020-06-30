// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "diff_drive_controller/urdf_util.hpp"

namespace urdf_util
{
double euclidean_of_vectors(const urdf::Vector3 & vec1, const urdf::Vector3 & vec2)
{
  return sqrt(
    std::pow(vec1.x - vec2.x, 2) + std::pow(vec1.y - vec2.y, 2) + std::pow(vec1.z - vec2.z, 2));
}

/*
* \brief Check that a link exists and has a geometry collision.
* \param link The link
* \return true if the link has a collision element with geometry
*/
bool has_collision_geometry(const urdf::LinkConstSharedPtr & link, const rclcpp::Logger & logger)
{
  if (!link) {
    RCLCPP_ERROR(logger, "Link pointer is null.");
    return false;
  }

  if (!link->collision) {
    RCLCPP_ERROR_STREAM(
      logger,
      "Link " <<
        link->name <<
        " does not have collision description. Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry) {
    RCLCPP_ERROR_STREAM(
      logger, "Link " << link->name <<
        " does not have collision geometry description. Add collision geometry "
        "description for link to urdf.");
    return false;
  }
  return true;
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
bool is_cylinder(const urdf::LinkConstSharedPtr & link, const rclcpp::Logger & logger)
{
  if (!has_collision_geometry(link, logger)) {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER) {
    RCLCPP_DEBUG_STREAM(logger, "Link " << link->name << " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Check if the link is modeled as a sphere
 * \param link Link
 * \return true if the link is modeled as a Sphere; false otherwise
 */

bool is_sphere(const urdf::LinkConstSharedPtr & link, const rclcpp::Logger & logger)
{
  if (!has_collision_geometry(link, logger)) {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::SPHERE) {
    RCLCPP_DEBUG_STREAM(logger, "Link " << link->name << " does not have sphere geometry");
    return false;
  }

  return true;
}

bool get_wheel_radius(
  const urdf::LinkConstSharedPtr & wheel_link, double & wheel_radius, const rclcpp::Logger & logger)
{
  if (is_cylinder(wheel_link, logger)) {
    wheel_radius = (static_cast<urdf::Cylinder *>(wheel_link->collision->geometry.get()))->radius;
    return true;
  } else if (is_sphere(wheel_link, logger)) {
    wheel_radius = (static_cast<urdf::Sphere *>(wheel_link->collision->geometry.get()))->radius;
    return true;
  }

  RCLCPP_ERROR_STREAM(
    logger, "Wheel link " << wheel_link->name << " is NOT modeled as a cylinder or sphere!");
  return false;
}
}  // namespace urdf_util
