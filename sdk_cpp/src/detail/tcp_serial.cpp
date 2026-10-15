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

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

#include <Robotiq/detail/default_logger.hpp>
#include <Robotiq/gripper/logger.hpp>
#include <Robotiq/gripper/serial_io_exception.hpp>

namespace Robotiq::detail {
namespace {
// Linux reports a broken pipe through MSG_NOSIGNAL; macOS through SO_NOSIGPIPE.
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif
} // namespace

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
      if(!waitReady(POLLOUT, deadline))
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

   std::vector<uint8_t> data(size);
   size_t received = 0;
   const auto deadline = Clock::now() + timeout;
   while(received < size)
   {
      const ssize_t count = ::recv(_socket, data.data() + received, size - received, 0);
      if(count > 0)
      {
         received += static_cast<size_t>(count);
         continue;
      }
      if(count == 0)
      {
         fail("the peer closed the connection");
      }
      if(errno == EINTR)
      {
         continue;
      }
      if(errno != EAGAIN && errno != EWOULDBLOCK)
      {
         fail(std::string("reading failed: ") + std::strerror(errno));
      }
      // A short read is how a timeout is reported; the link stays open.
      if(timeout.count() == 0 || !waitReady(POLLIN, deadline))
      {
         break;
      }
   }
   data.resize(received);
   return data;
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

   size_t sent = 0;
   const auto deadline = Clock::now() + _config.timeout;
   while(sent < data.size())
   {
      const ssize_t count = ::send(_socket, data.data() + sent, data.size() - sent, kSendFlags);
      if(count > 0)
      {
         sent += static_cast<size_t>(count);
         continue;
      }
      if(count < 0 && errno == EINTR)
      {
         continue;
      }
      if(count < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
      {
         fail(std::string("writing failed: ") + std::strerror(errno));
      }
      if(!waitReady(POLLOUT, deadline))
      {
         fail("writing timed out after " + std::to_string(sent) + " of " + std::to_string(data.size()) + " bytes");
      }
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

bool TcpSerial::waitReady(short events, Clock::time_point deadline)
{
   while(true)
   {
      const auto remaining = deadline - Clock::now();
      if(remaining <= Clock::duration::zero())
      {
         return false;
      }
      // Round up, so a sub-millisecond rest does not turn into a busy loop.
      const auto remainingMs = std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
      const int waitMs = static_cast<int>(std::min<int64_t>(remainingMs, std::numeric_limits<int>::max()));
      pollfd descriptor{_socket, events, 0};
      const int result = ::poll(&descriptor, 1, waitMs);
      if(result < 0)
      {
         if(errno == EINTR)
         {
            continue;
         }
         fail(std::string("waiting on the socket failed: ") + std::strerror(errno));
      }
      if(result == 0)
      {
         continue;
      }
      // Buffered bytes stay readable after the peer has closed its end.
      if((descriptor.revents & events) != 0)
      {
         return true;
      }
      fail("the connection was closed or reported an error");
   }
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
