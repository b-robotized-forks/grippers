// Copyright (c) 2026 b»robotized group
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

//! \brief Serial implementation over a pseudo-terminal (Linux).
//! A bridge such as socat exposes a remote RS-485 bus as /dev/pts/N, often
//! through a link like /tmp/ttyUR (UR tool communication). libserialport
//! enumerates ports through sysfs, where pseudo-terminals have no entry, so
//! DefaultSerial cannot open them; this transport opens the path directly.
//! The baud rate is the bridge's; SerialConfig::baudrate is not applied.
//!
//! The exchange cycle opens a Serial once and never again. A restarted bridge
//! creates a new /dev/pts/N behind the same link, so after the other end has
//! closed, the next write() opens the path again, at most once per second.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Robotiq/gripper/serial.hpp>
#include <Robotiq/gripper/serial_config.hpp>

namespace Robotiq {
class Logger;
} // namespace Robotiq

namespace Robotiq::detail {
class PtySerial : public Serial
{
public:
   // \param logger Log sink; pass null to use the default stderr logger.
   explicit PtySerial(SerialConfig config, std::shared_ptr<Logger> logger = nullptr);
   ~PtySerial() override;

   PtySerial(const PtySerial&) = delete;
   PtySerial& operator=(const PtySerial&) = delete;

   void open() override;

   [[nodiscard]] bool isOpen() const override;
   void close() override;

   // Collects up to \p size bytes; returns fewer only when \p timeout expires.
   // A zero timeout returns the bytes already received without waiting.
   [[nodiscard]] std::vector<uint8_t> read(size_t size, std::chrono::milliseconds timeout) override;
   void write(const std::vector<uint8_t>& data) override;

   [[nodiscard]] std::chrono::milliseconds getTimeout() const override;

   // Link parameters this connection was constructed with.
   [[nodiscard]] const SerialConfig& getConfig() const;

   // Minimum time between two attempts to open the path again.
   static constexpr std::chrono::milliseconds kReopenInterval{1000};

private:
   using Clock = std::chrono::steady_clock;

   // Closes the terminal and throws SerialIOException.
   [[noreturn]] void fail(const std::string& message);

   int _fd = -1;
   SerialConfig _config;
   std::shared_ptr<Logger> _logger;
   Clock::time_point _lastOpenAttempt{};
   bool _openedBefore = false;
};

// Whether \p port, after following links, is a pseudo-terminal (/dev/pts/N).
[[nodiscard]] bool isPseudoTerminal(const std::string& port);

} // namespace Robotiq::detail
