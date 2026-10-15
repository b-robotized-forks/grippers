// Copyright (c) 2026 b»robotized group
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

//! \brief Serial implementation over a raw Modbus RTU over TCP link (POSIX).
//! The peer forwards the bytes to the gripper's RS-485 bus, e.g. the UR Tool
//! Communication Forwarder. Link parameters are fixed at construction
//! (TcpConfig).
//!
//! The exchange cycle opens a Serial once and never again, and returns to
//! Operational on the first successful exchange after a fault. So this
//! transport recovers by itself: an I/O error or a closed peer closes the
//! socket, and the next write() connects again, at most once per
//! TcpConfig::reconnectInterval. A new connection also drops any late reply
//! bytes from the lost one.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Robotiq/gripper/serial.hpp>
#include <Robotiq/gripper/tcp_config.hpp>

namespace Robotiq {
class Logger;
} // namespace Robotiq

namespace Robotiq::detail {
class TcpSerial : public Serial
{
public:
   // \param logger Log sink; pass null to use the default stderr logger.
   explicit TcpSerial(TcpConfig config, std::shared_ptr<Logger> logger = nullptr);
   ~TcpSerial() override;

   TcpSerial(const TcpSerial&) = delete;
   TcpSerial& operator=(const TcpSerial&) = delete;

   void open() override;

   [[nodiscard]] bool isOpen() const override;
   void close() override;

   // Collects up to \p size bytes; returns fewer only when \p timeout expires.
   // A zero timeout returns the bytes already received without waiting.
   [[nodiscard]] std::vector<uint8_t> read(size_t size, std::chrono::milliseconds timeout) override;
   void write(const std::vector<uint8_t>& data) override;

   [[nodiscard]] std::chrono::milliseconds getTimeout() const override;

   // Link parameters this connection was constructed with.
   [[nodiscard]] const TcpConfig& getConfig() const;

private:
   using Clock = std::chrono::steady_clock;

   // Waits until the socket is ready for \p events; false on timeout.
   [[nodiscard]] bool waitReady(short events, Clock::time_point deadline);
   // Closes the socket and throws SerialIOException.
   [[noreturn]] void fail(const std::string& message);
   [[nodiscard]] std::string endpoint() const;

   int _socket = -1;
   TcpConfig _config;
   std::shared_ptr<Logger> _logger;
   Clock::time_point _lastConnectAttempt{};
   bool _connectedBefore = false;
};
} // namespace Robotiq::detail
