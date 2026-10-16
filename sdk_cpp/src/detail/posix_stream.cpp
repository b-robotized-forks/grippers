// Copyright (c) 2026 b»robotized group
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include "posix_stream.hpp"

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <limits>
#include <string>

namespace Robotiq::detail::posix {
namespace {
// Linux reports a broken pipe through MSG_NOSIGNAL; macOS through SO_NOSIGPIPE,
// which TcpSerial sets on the socket.
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif

ssize_t readSome(int fd, Kind kind, uint8_t* buffer, size_t size)
{
   return kind == Kind::Socket ? ::recv(fd, buffer, size, 0) : ::read(fd, buffer, size);
}

ssize_t writeSome(int fd, Kind kind, const uint8_t* buffer, size_t size)
{
   return kind == Kind::Socket ? ::send(fd, buffer, size, kSendFlags) : ::write(fd, buffer, size);
}

[[noreturn]] void throwErrno(const std::string& what)
{
   throw StreamError(what + ": " + std::strerror(errno));
}
} // namespace

bool waitReady(int fd, short events, Clock::time_point deadline)
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
      pollfd descriptor{fd, events, 0};
      const int result = ::poll(&descriptor, 1, waitMs);
      if(result < 0)
      {
         if(errno == EINTR)
         {
            continue;
         }
         throwErrno("waiting for the link failed");
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
      throw StreamError("the link was closed or reported an error");
   }
}

std::vector<uint8_t> readUpTo(int fd, Kind kind, size_t size, std::chrono::milliseconds timeout)
{
   std::vector<uint8_t> data(size);
   size_t received = 0;
   const auto deadline = Clock::now() + timeout;
   while(received < size)
   {
      const ssize_t count = readSome(fd, kind, data.data() + received, size - received);
      if(count > 0)
      {
         received += static_cast<size_t>(count);
         continue;
      }
      // A closed socket reads 0 bytes; a pseudo-terminal whose other end has
      // closed fails with EIO.
      if(count == 0 || errno == EIO)
      {
         throw StreamError("the peer closed the link");
      }
      if(errno == EINTR)
      {
         continue;
      }
      if(errno != EAGAIN && errno != EWOULDBLOCK)
      {
         throwErrno("reading failed");
      }
      // A short read is how a timeout is reported; the link stays open.
      if(timeout.count() == 0 || !waitReady(fd, POLLIN, deadline))
      {
         break;
      }
   }
   data.resize(received);
   return data;
}

void writeAll(int fd, Kind kind, const std::vector<uint8_t>& data, std::chrono::milliseconds timeout)
{
   size_t sent = 0;
   const auto deadline = Clock::now() + timeout;
   while(sent < data.size())
   {
      const ssize_t count = writeSome(fd, kind, data.data() + sent, data.size() - sent);
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
         throwErrno("writing failed");
      }
      if(!waitReady(fd, POLLOUT, deadline))
      {
         throw StreamError("writing timed out after " + std::to_string(sent) + " of " + std::to_string(data.size())
                           + " bytes");
      }
   }
}
} // namespace Robotiq::detail::posix
