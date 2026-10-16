// Copyright (c) 2026 b»robotized group
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <Robotiq/detail/pty_serial.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

#include <Robotiq/detail/default_logger.hpp>
#include <Robotiq/gripper/logger.hpp>
#include <Robotiq/gripper/serial_io_exception.hpp>

#include "posix_stream.hpp"

namespace Robotiq::detail {

bool isPseudoTerminal(const std::string& port)
{
   char resolved[PATH_MAX];
   if(port.empty() || ::realpath(port.c_str(), resolved) == nullptr)
   {
      return false;
   }
   return std::string(resolved).rfind("/dev/pts/", 0) == 0;
}

PtySerial::PtySerial(SerialConfig config, std::shared_ptr<Logger> logger)
   : _config(std::move(config))
   , _logger(logger ? std::move(logger) : makeDefaultLogger())
{
}

PtySerial::~PtySerial()
{
   PtySerial::close();
}

void PtySerial::open()
{
   if(_fd >= 0)
   {
      return;
   }
   _lastOpenAttempt = Clock::now();

   _fd = ::open(_config.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
   if(_fd < 0)
   {
      fail(std::string("opening failed: ") + std::strerror(errno));
   }
   // Raw bytes: no line editing, echo or character translation on the frames.
   termios attributes{};
   if(::tcgetattr(_fd, &attributes) < 0)
   {
      fail(std::string("reading the terminal settings failed: ") + std::strerror(errno));
   }
   ::cfmakeraw(&attributes);
   if(::tcsetattr(_fd, TCSANOW, &attributes) < 0)
   {
      fail(std::string("setting the terminal to raw failed: ") + std::strerror(errno));
   }

   _logger->log(Logger::Level::Info,
                (_openedBefore ? "reopened pseudo-terminal " : "opened pseudo-terminal ") + _config.port
                   + "; the baud rate is set by the bridge");
   _openedBefore = true;
}

bool PtySerial::isOpen() const
{
   return _fd >= 0;
}

void PtySerial::close()
{
   if(_fd >= 0)
   {
      ::close(_fd);
      _fd = -1;
   }
}

std::vector<uint8_t> PtySerial::read(size_t size, std::chrono::milliseconds timeout)
{
   if(_fd < 0)
   {
      throw SerialIOException("read called on closed pseudo-terminal " + _config.port);
   }
   try
   {
      return posix::readUpTo(_fd, posix::Kind::Terminal, size, timeout);
   }
   catch(const posix::StreamError& e)
   {
      fail(e.what());
   }
}

void PtySerial::write(const std::vector<uint8_t>& data)
{
   if(_fd < 0)
   {
      if(Clock::now() - _lastOpenAttempt < kReopenInterval)
      {
         throw SerialIOException("pseudo-terminal " + _config.port + " is closed; waiting before opening it again");
      }
      open();
   }
   try
   {
      posix::writeAll(_fd, posix::Kind::Terminal, data, _config.timeout);
   }
   catch(const posix::StreamError& e)
   {
      fail(e.what());
   }
}

std::chrono::milliseconds PtySerial::getTimeout() const
{
   return _config.timeout;
}

const SerialConfig& PtySerial::getConfig() const
{
   return _config;
}

void PtySerial::fail(const std::string& message)
{
   close();
   throw SerialIOException(_config.port + ": " + message);
}
} // namespace Robotiq::detail
