// Copyright (c) 2026 b»robotized group
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace Robotiq {

//! \ingroup connection
//! \brief Parameters of a raw Modbus RTU over TCP link.
//!
//! The peer forwards the bytes unchanged to the gripper's RS-485 bus, e.g. the
//! UR Tool Communication Forwarder on a Universal Robots arm. The frames keep
//! their RTU form with CRC; this is not Modbus TCP. Baud rate, parity and stop
//! bits are configured on the peer, not here.
struct TcpConfig
{
   //! \brief Numeric IPv4 or IPv6 address of the peer. No DNS lookup is made.
   std::string host;

   //! \brief TCP port of the peer (UR Tool Communication Forwarder: 54321).
   uint16_t port = 54321;

   //! Per-transaction timeout for connecting, blocking reads and writes.
   std::chrono::milliseconds timeout{500};

   //! \brief Minimum time between two connection attempts after the link is lost.
   //!
   //! The exchange cycle calls write() at its own rate; this keeps a missing
   //! peer from being dialed on every cycle.
   std::chrono::milliseconds reconnectInterval{1000};
};
} // namespace Robotiq
