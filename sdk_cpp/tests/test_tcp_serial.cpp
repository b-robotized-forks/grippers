// Copyright (c) 2026 b»robotized group
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

// TcpSerial against loopback peers: a raw socket for the byte-level contract,
// and the fake gripper behind a TCP socket for the exchange cycle on top.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <Robotiq/detail/tcp_serial.hpp>
#include <Robotiq/gripper.hpp>
#include <Robotiq/gripper/connection_config.hpp>
#include <Robotiq/gripper/logger.hpp>
#include <Robotiq/gripper/modbus_client.hpp>
#include <Robotiq/gripper/serial_io_exception.hpp>

#include "fake/gripper_server.hpp"
#include "fake/register_model.hpp"
#include "test_utils.hpp"

namespace Robotiq::test {
using detail::TcpSerial;
using namespace std::chrono_literals;

namespace {
// A send to a closed peer must not raise SIGPIPE: Linux takes MSG_NOSIGNAL,
// macOS SO_NOSIGPIPE on the socket (see noSigpipe()).
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif

int noSigpipe(int socket)
{
#ifdef SO_NOSIGPIPE
   const int enabled = 1;
   setsockopt(socket, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled));
#endif
   return socket;
}

// A loopback socket bound to a free port. Without \p listen, connections to
// it are refused.
int makeListener(bool listen = true)
{
   const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
   sockaddr_in address{};
   address.sin_family = AF_INET;
   address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
   EXPECT_EQ(::bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0);
   if(listen)
   {
      EXPECT_EQ(::listen(listener, 1), 0);
   }
   return listener;
}

uint16_t portOf(int socket)
{
   sockaddr_in address{};
   socklen_t length = sizeof(address);
   EXPECT_EQ(::getsockname(socket, reinterpret_cast<sockaddr*>(&address), &length), 0);
   return ntohs(address.sin_port);
}

TcpConfig loopbackConfig(uint16_t port)
{
   TcpConfig config;
   config.host = "127.0.0.1";
   config.port = port;
   config.timeout = 200ms;
   config.reconnectInterval = 0ms;
   return config;
}

void sendAll(int socket, const std::vector<uint8_t>& data)
{
   ASSERT_EQ(::send(socket, data.data(), data.size(), kSendFlags), static_cast<ssize_t>(data.size()));
}

std::vector<uint8_t> receive(int socket, size_t size)
{
   std::vector<uint8_t> data(size);
   const timeval timeout{2, 0};
   setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
   const ssize_t count = ::recv(socket, data.data(), size, MSG_WAITALL);
   data.resize(count > 0 ? static_cast<size_t>(count) : 0);
   return data;
}

// The fake gripper behind a TCP socket, as a bridge to the RS-485 bus would
// present it. Requests are delimited by a pause in the byte stream.
class FakeGripperTcpServer
{
public:
   FakeGripperTcpServer()
      : _model(std::make_shared<NullLogger>())
      , _server(_model, kSlaveAddress)
      , _listener(makeListener())
      , _thread([this] { run(); })
   {
   }

   ~FakeGripperTcpServer()
   {
      _running.store(false);
      _thread.join();
      closePeer();
      ::close(_listener);
   }

   FakeGripperTcpServer(const FakeGripperTcpServer&) = delete;
   FakeGripperTcpServer& operator=(const FakeGripperTcpServer&) = delete;

   [[nodiscard]] uint16_t port() const { return portOf(_listener); }
   [[nodiscard]] int connections() const { return _connections.load(); }

   // Closes the current connection, as a restarted bridge would.
   void dropConnection() { _dropRequested.store(true); }

private:
   void run()
   {
      while(_running.load())
      {
         if(_dropRequested.exchange(false))
         {
            closePeer();
         }
         pollfd listener{_listener, POLLIN, 0};
         if(::poll(&listener, 1, 0) > 0)
         {
            closePeer();
            _peer = noSigpipe(::accept(_listener, nullptr, nullptr));
            _connections.fetch_add(1);
         }
         if(_peer < 0)
         {
            std::this_thread::sleep_for(1ms);
            continue;
         }
         const std::vector<uint8_t> request = readRequest();
         if(request.empty())
         {
            continue;
         }
         _server.deliver(request);
         const std::vector<uint8_t> reply = _server.drain(256);
         if(::send(_peer, reply.data(), reply.size(), kSendFlags) != static_cast<ssize_t>(reply.size()))
         {
            closePeer();
         }
      }
   }

   std::vector<uint8_t> readRequest()
   {
      std::vector<uint8_t> request;
      int waitMs = 5;
      while(true)
      {
         pollfd peer{_peer, POLLIN, 0};
         if(::poll(&peer, 1, waitMs) <= 0)
         {
            return request;
         }
         uint8_t buffer[256];
         const ssize_t count = ::recv(_peer, buffer, sizeof(buffer), 0);
         if(count <= 0)
         {
            closePeer();
            return {};
         }
         request.insert(request.end(), buffer, buffer + count);
         // Once a request has started, a short pause ends it.
         waitMs = 3;
      }
   }

   void closePeer()
   {
      if(_peer >= 0)
      {
         ::close(_peer);
         _peer = -1;
      }
   }

   fake::RegisterModel _model;
   fake::GripperServer _server;
   int _listener;
   int _peer = -1;
   std::atomic<bool> _running{true};
   std::atomic<bool> _dropRequested{false};
   std::atomic<int> _connections{0};
   std::thread _thread;
};
} // namespace

class TestTcpSerial : public ::testing::Test
{
protected:
   TestTcpSerial()
      : listener(makeListener())
      , serial(loopbackConfig(portOf(listener)), std::make_shared<NullLogger>())
   {
   }

   ~TestTcpSerial() override
   {
      serial.close();
      closePeer();
      ::close(listener);
   }

   void connect()
   {
      serial.open();
      peer = noSigpipe(::accept(listener, nullptr, nullptr));
      ASSERT_GE(peer, 0);
   }

   void closePeer()
   {
      if(peer >= 0)
      {
         ::close(peer);
         peer = -1;
      }
   }

   int listener;
   int peer = -1;
   TcpSerial serial;
};

TEST(TestTcpSerialConfig, rejects_a_host_that_is_not_a_numeric_address)
{
   TcpConfig config;
   config.host = "localhost";
   TcpSerial serial(config, std::make_shared<NullLogger>());
   EXPECT_THROW(serial.open(), SerialIOException);
   EXPECT_FALSE(serial.isOpen());
}

TEST(TestTcpSerialConfig, throws_when_the_connection_is_refused)
{
   const int closedPort = makeListener(false);
   TcpSerial serial(loopbackConfig(portOf(closedPort)), std::make_shared<NullLogger>());
   EXPECT_THROW(serial.open(), SerialIOException);
   EXPECT_FALSE(serial.isOpen());
   ::close(closedPort);
}

TEST_F(TestTcpSerial, read_collects_a_fragmented_reply)
{
   connect();
   std::thread peerThread([this] {
      sendAll(peer, {0x09, 0x03});
      std::this_thread::sleep_for(20ms);
      sendAll(peer, {0x02, 0x00, 0xFF});
   });
   EXPECT_EQ(serial.read(5, 200ms), (std::vector<uint8_t>{0x09, 0x03, 0x02, 0x00, 0xFF}));
   peerThread.join();
}

TEST_F(TestTcpSerial, read_returns_fewer_bytes_on_timeout_and_stays_open)
{
   connect();
   sendAll(peer, {0x01, 0x02});
   const auto start = std::chrono::steady_clock::now();
   EXPECT_EQ(serial.read(5, 30ms), (std::vector<uint8_t>{0x01, 0x02}));
   EXPECT_GE(std::chrono::steady_clock::now() - start, 25ms);
   EXPECT_TRUE(serial.isOpen());
}

TEST_F(TestTcpSerial, zero_timeout_returns_received_bytes_without_waiting)
{
   connect();
   sendAll(peer, {0x01, 0x02, 0x03});
   std::this_thread::sleep_for(20ms);
   EXPECT_EQ(serial.read(256, 0ms), (std::vector<uint8_t>{0x01, 0x02, 0x03}));
   const auto start = std::chrono::steady_clock::now();
   EXPECT_TRUE(serial.read(256, 0ms).empty());
   EXPECT_LT(std::chrono::steady_clock::now() - start, 10ms);
}

TEST_F(TestTcpSerial, write_sends_all_bytes)
{
   connect();
   const std::vector<uint8_t> request{0x09, 0x03, 0x07, 0xD0, 0x00, 0x03, 0x04, 0x0E};
   serial.write(request);
   EXPECT_EQ(receive(peer, request.size()), request);
}

TEST_F(TestTcpSerial, a_closed_peer_throws_and_closes_the_link)
{
   connect();
   closePeer();
   EXPECT_THROW((void)serial.read(1, 200ms), SerialIOException);
   EXPECT_FALSE(serial.isOpen());
}

TEST_F(TestTcpSerial, write_reconnects_after_the_link_was_lost)
{
   connect();
   closePeer();
   EXPECT_THROW((void)serial.read(1, 200ms), SerialIOException);

   serial.write({0x0A, 0x0B});
   peer = ::accept(listener, nullptr, nullptr);
   ASSERT_GE(peer, 0);
   EXPECT_TRUE(serial.isOpen());
   EXPECT_EQ(receive(peer, 2), (std::vector<uint8_t>{0x0A, 0x0B}));
}

TEST(TestTcpSerialConfig, write_waits_for_the_reconnect_interval)
{
   const int listener = makeListener();
   TcpConfig config = loopbackConfig(portOf(listener));
   config.reconnectInterval = 10s;
   TcpSerial serial(config, std::make_shared<NullLogger>());
   serial.open();
   const int peer = ::accept(listener, nullptr, nullptr);
   ::close(peer);
   EXPECT_THROW((void)serial.read(1, 200ms), SerialIOException);

   EXPECT_THROW(serial.write({0x01}), SerialIOException);
   EXPECT_FALSE(serial.isOpen());
   ::close(listener);
}

TEST(TestTcpSerialGripper, modbus_client_exchanges_with_the_fake_gripper)
{
   FakeGripperTcpServer server;
   GripperModbusClient client(
      std::make_unique<TcpSerial>(loopbackConfig(server.port()), std::make_shared<NullLogger>()),
      kSlaveAddress,
      std::make_shared<NullLogger>());

   GripperCommand command;
   command.action.set(ActionRequestBit::Activate, true);
   client.writeCommand(command);

   EXPECT_TRUE(client.readStatus().gripperStatus.activated());
}

TEST(TestTcpSerialGripper, gripper_recovers_after_the_connection_is_dropped)
{
   FakeGripperTcpServer server;
   ConnectionConfig config;
   config.tcp = loopbackConfig(server.port());
   config.tcp->reconnectInterval = 50ms;
   Gripper gripper(config, std::make_shared<NullLogger>());
   ASSERT_EQ(gripper.connectionState(), ConnectionState::Operational);

   server.dropConnection();
   EXPECT_TRUE(pollFor([&] { return gripper.connectionState() == ConnectionState::Faulted; }, 5s));

   EXPECT_TRUE(pollFor([&] { return gripper.connectionState() == ConnectionState::Operational; }, 5s));
   EXPECT_EQ(server.connections(), 2);
}
} // namespace Robotiq::test
