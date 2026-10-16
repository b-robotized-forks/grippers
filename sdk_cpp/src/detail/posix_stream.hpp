// Copyright (c) 2026 b»robotized group
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

// Byte I/O on a non-blocking POSIX file descriptor, shared by the socket
// (TcpSerial) and pseudo-terminal (PtySerial) transports. The Serial
// implementations add their endpoint to a StreamError and close the link.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace Robotiq::detail::posix {

using Clock = std::chrono::steady_clock;

//! An I/O failure after which the descriptor is unusable.
class StreamError : public std::runtime_error
{
public:
   using std::runtime_error::runtime_error;
};

//! How the descriptor transfers bytes: a socket uses send()/recv(), so a
//! closed peer cannot raise SIGPIPE.
enum class Kind
{
   Socket,
   Terminal,
};

//! Wait until \p fd is ready for \p events; false when \p deadline passes.
//! \throws StreamError when the peer has gone or the descriptor fails.
[[nodiscard]] bool waitReady(int fd, short events, Clock::time_point deadline);

//! Collect up to \p size bytes; fewer only when \p timeout expires. A zero
//! timeout returns the bytes already received without waiting.
//! \throws StreamError when the peer has gone or reading fails.
[[nodiscard]] std::vector<uint8_t> readUpTo(int fd, Kind kind, size_t size, std::chrono::milliseconds timeout);

//! Write all of \p data within \p timeout.
//! \throws StreamError on a timeout or when writing fails.
void writeAll(int fd, Kind kind, const std::vector<uint8_t>& data, std::chrono::milliseconds timeout);

} // namespace Robotiq::detail::posix
