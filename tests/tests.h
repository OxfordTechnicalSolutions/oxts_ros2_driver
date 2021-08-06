/**
 * \file tests.h
 * Contains anything which is common to all the unit tests.
 */

#ifndef TESTS_H
#define TESTS_H

#include <memory>

#include "oxts_driver/driver.hpp"

namespace tests {

/**
 * @return A new OxtsDriver for use in the unit tests (i.e. configured to use an .ncom file).
 */
std::unique_ptr<oxts_driver::OxtsDriver> newDriver();

}

#endif // TESTS_H