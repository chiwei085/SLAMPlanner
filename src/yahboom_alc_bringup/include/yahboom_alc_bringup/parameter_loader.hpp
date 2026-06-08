#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rik_sd/struct_descriptor.hpp>

#include <cstdint>
#include <string>
#include <type_traits>

namespace yahboom_alc_bringup
{

template <typename>
inline constexpr bool dependent_false_v = false;

template <typename T>
void loadDeclaredParametersInto(rclcpp::Node& node, T& config) {
    rik_sd::for_each(config, [&node](const auto& field, auto& value) {
        const auto& core = rik_sd::core_field(field);
        const std::string parameter_name{core.name_sv()};
        node.declare_parameter(parameter_name, value);

        const auto parameter = node.get_parameter(parameter_name);
        using value_t = rik_sd::remove_cvref_t<decltype(value)>;
        if constexpr (std::is_same_v<value_t, std::string>) {
            value = parameter.as_string();
        }
        else if constexpr (std::is_same_v<value_t, double>) {
            value = parameter.as_double();
        }
        else if constexpr (std::is_same_v<value_t, bool>) {
            value = parameter.as_bool();
        }
        else if constexpr (std::is_same_v<value_t, int>) {
            value = static_cast<int>(parameter.as_int());
        }
        else if constexpr (std::is_same_v<value_t, std::int64_t>) {
            value = parameter.as_int();
        }
        else {
            static_assert(dependent_false_v<value_t>,
                          "Unsupported ROS parameter field type");
        }
    });
}

template <typename T>
T loadDeclaredParameters(rclcpp::Node& node, T defaults = {}) {
    loadDeclaredParametersInto(node, defaults);
    return defaults;
}

}  // namespace yahboom_alc_bringup
