#include <cstring>
#include <chrono>
#include <future>
#include <array>
#include <memory>
#include <boost/test/unit_test.hpp>

#include "oxts_driver/udp_server_client.h"

using namespace std::literals;

namespace tests::udp_server_client {

const short PORT_NUMBER = 12345;

std::unique_ptr<networking_udp::client> client;
std::unique_ptr<networking_udp::server> server;

constexpr int MESSAGE_LEN = 13;
networking_udp::byte MESSAGE[MESSAGE_LEN] = "hello, world";

struct Fixture {
    Fixture() {
        client = std::make_unique<networking_udp::client>(PORT_NUMBER);
        server = std::make_unique<networking_udp::server>();
        server->set_remote_endpoint("127.0.0.1", PORT_NUMBER);
    }
};

BOOST_AUTO_TEST_SUITE(udp_server_client, * boost::unit_test::fixture<Fixture>())

BOOST_AUTO_TEST_CASE(sendReceive) {
    auto future = std::async(std::launch::async, []() {
        auto received = std::array<networking_udp::byte, MESSAGE_LEN>{};
        client->receive(received.data(), MESSAGE_LEN);
        return received;
    });
    std::this_thread::sleep_for(100ms); // race conditions; just in case

    server->send(MESSAGE, MESSAGE_LEN);

    auto received = future.get();
    BOOST_CHECK(std::strncmp((const char*)received.data(), (const char*)MESSAGE, MESSAGE_LEN) == 0);
}

BOOST_AUTO_TEST_CASE(sendTo) {
    using namespace boost::asio::ip;

    auto future = std::async(std::launch::async, []() {
        auto received = std::array<networking_udp::byte, MESSAGE_LEN>{};
        client->receive(received.data(), MESSAGE_LEN);
        return received;
    });
    std::this_thread::sleep_for(100ms); // race conditions; just in case

    server->send_to(
        MESSAGE, MESSAGE_LEN,
        udp::endpoint{address::from_string("127.0.0.1"), PORT_NUMBER}
    );

    auto received = future.get();
    BOOST_CHECK(std::strncmp((const char*)received.data(), (const char*)MESSAGE, MESSAGE_LEN) == 0);
}

BOOST_AUTO_TEST_SUITE_END()

}