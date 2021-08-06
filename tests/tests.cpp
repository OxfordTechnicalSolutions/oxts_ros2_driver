#define BOOST_TEST_MAIN

#include <boost/test/unit_test.hpp>
#include <memory>

#include "tests.h"
#include "oxts_driver/driver.hpp"

namespace tests {

std::unique_ptr<oxts_driver::OxtsDriver> newDriver() {
    rclcpp::NodeOptions options;
    options.append_parameter_override("ncom", std::filesystem::path{__FILE__}.replace_filename("test.ncom"));
    options.append_parameter_override("ncom_rate", 250);
    return std::make_unique<oxts_driver::OxtsDriver>(options);
}

}