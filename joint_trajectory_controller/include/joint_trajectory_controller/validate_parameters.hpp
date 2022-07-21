#include <set>
#include <sstream>

Result validate_string_is_one_of(rclcpp::Parameter const& parameter, std::set<std::string> collection) {
  auto const& string_param = parameter.as_string();

  if (collection.find(string_param) == collection.end()) {
    std::stringstream ss;
    for (auto const& c : collection) ss << c << ", ";
    return ERROR(
      "The parameter (%s) with the value (%s) not in the valid values: [%s]",
      parameter.get_name(), string_param, ss.str());
  }

  return OK;
}

auto validate_interpolation_method = [](auto const& parameter) {
  return validate_string_is_one_of(parameter, {"none", "splines"});
};
