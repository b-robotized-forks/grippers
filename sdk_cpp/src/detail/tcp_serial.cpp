// Copyright (c) 2026 b»robotized group
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <Robotiq/detail/tcp_serial.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <utility>

#include <Robotiq/detail/default_logger.hpp>
#include <Robotiq/gripper/logger.hpp>
#include <Robotiq/gripper/serial_io_exception.hpp>

#include "posix_stream.hpp"

namespace Robotiq::detail {

TcpSerial::TcpSerial(TcpConfig config, std::shared_ptr<Logger> logger)
   : _config(std::move(config))
   , _logger(logger ? std::move(logger) : makeDefaultLogger())
{
}

TcpSerial::~TcpSerial()
{
   TcpSerial::close();
}

void TcpSerial::open()
{
   if(_socket >= 0)
   {
      return;
   }

   sockaddr_storage address{};
   socklen_t addressLength = 0;
   auto* ipv4 = reinterpret_cast<sockaddr_in*>(&address);
   auto* ipv6 = reinterpret_cast<sockaddr_in6*>(&address);
   if(inet_pton(AF_INET, _config.host.c_str(), &ipv4->sin_addr) == 1)
   {
      ipv4->sin_family = AF_INET;
      ipv4->sin_port = htons(_config.port);
      addressLength = sizeof(sockaddr_in);
   }
   else if(inet_pton(AF_INET6, _config.host.c_str(), &ipv6->sin6_addr) == 1)
   {
      ipv6->sin6_family = AF_INET6;
      ipv6->sin6_port = htons(_config.port);
      addressLength = sizeof(sockaddr_in6);
   }
   else
   {
      throw SerialIOException("TCP host '" + _config.host + "' is not a numeric IPv4 or IPv6 address");
   }

   _lastConnectAttempt = Clock::now();
   const auto deadline = _lastConnectAttempt + _config.timeout;

   _socket = ::socket(address.ss_family, SOCK_STREAM, 0);
   if(_socket < 0)
   {
      fail(std::string("creating the socket failed: ") + std::strerror(errno));
   }
   const int flags = fcntl(_socket, F_GETFL, 0);
   const int enabled = 1;
   if(flags < 0 || fcntl(_socket, F_SETFL, flags | O_NONBLOCK) < 0 || fcntl(_socket, F_SETFD, FD_CLOEXEC) < 0
      || setsockopt(_socket, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled)) < 0)
   {
      fail(std::string("configuring the socket failed: ") + std::strerror(errno));
   }
#ifdef SO_NOSIGPIPE
   if(setsockopt(_socket, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled)) < 0)
   {
      fail(std::string("configuring the socket failed: ") + std::strerror(errno));
   }
#endif

   if(::connect(_socket, reinterpret_cast<sockaddr*>(&address), addressLength) < 0)
   {
      if(errno != EINPROGRESS)
      {
         fail(std::string("connecting failed: ") + std::strerror(errno));
      }
      bool connected = false;
      try
      {
         connected = posix::waitReady(_socket, POLLOUT, deadline);
      }
      catch(const posix::StreamError& e)
      {
         fail(std::string("connecting failed: ") + e.what());
      }
      if(!connected)
      {
         fail("connecting timed out");
      }
      int error = 0;
      socklen_t errorLength = sizeof(error);
      if(getsockopt(_socket, SOL_SOCKET, SO_ERROR, &error, &errorLength) < 0)
      {
         error = errno;
      }
      if(error != 0)
      {
         fail(std::string("connecting failed: ") + std::strerror(error));
      }
   }

   _logger->log(Logger::Level::Info, (_connectedBefore ? "reconnected to " : "connected to ") + endpoint());
   _connectedBefore = true;
}

bool TcpSerial::isOpen() const
{
   return _socket >= 0;
}

void TcpSerial::close()
{
   if(_socket >= 0)
   {
      ::close(_socket);
      _socket = -1;
   }
}

std::vector<uint8_t> TcpSerial::read(size_t size, std::chrono::milliseconds timeout)
{
   if(_socket < 0)
   {
      throw SerialIOException("read called on a closed connection to " + endpoint());
   }

   try
   {
      return posix::readUpTo(_socket, posix::Kind::Socket, size, timeout);
   }
   catch(const posix::StreamError& e)
   {
      fail(e.what());
   }
}

void TcpSerial::write(const std::vector<uint8_t>& data)
{
   if(_socket < 0)
   {
      const auto sinceLastAttempt = Clock::now() - _lastConnectAttempt;
      if(sinceLastAttempt < _config.reconnectInterval)
      {
         throw SerialIOException("not connected to " + endpoint() + "; waiting before the next connection attempt");
      }
      open();
   }

   try
   {
      posix::writeAll(_socket, posix::Kind::Socket, data, _config.timeout);
   }
   catch(const posix::StreamError& e)
   {
      fail(e.what());
   }
}

std::chrono::milliseconds TcpSerial::getTimeout() const
{
   return _config.timeout;
}

const TcpConfig& TcpSerial::getConfig() const
{
   return _config;
}

void TcpSerial::fail(const std::string& message)
{
   close();
   throw SerialIOException(endpoint() + ": " + message);
}

std::string TcpSerial::endpoint() const
{
   const bool ipv6 = _config.host.find(':') != std::string::npos;
   return "tcp://" + (ipv6 ? "[" + _config.host + "]" : _config.host) + ":" + std::to_string(_config.port);
}
} // namespace Robotiq::detail
