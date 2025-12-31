
#include <limits>
#include <semantic_components/semantic_component_interface.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/detail/battery_state__struct.hpp>

namespace battery_state_broadcaster
{
class BatterySensor : public semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>
{
public:
  explicit BatterySensor(const std::string& name)
    : semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>(name, 1)
  {
    interface_names_.emplace_back(name_ + "/" + "voltage");
  }

  virtual ~BatterySensor() = default;

  double get_voltage()
  {
    voltage_ = state_interfaces_[0].get().get_optional().value();
    return voltage_;
  }

  bool get_values_as_message(sensor_msgs::msg::BatteryState& message)
  {
    get_voltage();
    message.voltage = static_cast<float>(voltage_);
    return true;
  }

private:
  double voltage_ = 0.0;
};
}  // namespace battery_state_broadcaster
