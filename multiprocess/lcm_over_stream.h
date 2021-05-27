#pragma once

#include <memory>
#include <string>

#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace multiprocess {

/// @file
/// An implementation of `DrakeLcmInterface` that operates over a stream; it
/// thus guarantees reliable in-order delivery at the cost of losing all
/// broadcast/multicast functionality.
///
/// This is largely redundant with `bot-lcm-tunnel`, but that code is LGPL and
/// so I can't edit in the layers I need to add features or make it work as a
/// library and rip out its UDP.
///
/// It is also quite similar to the (also GPL-encumbered) line protocol from
/// LCM's undocumented, incomplete, and unsupported "tcpq" provider, at least
/// as best I can understand it; LCM provides no documentation for that
/// provider or its protocol.  tcpq was intended as a many-to-many client
/// messaging system with a server brokering traffic, ie, replicating the
/// multicast semantics.  This provider simply does not attempt that, leaving
/// message brokering as a higher layer concern.
///
/// Here is the protocol, which is my best interpretation of LCM tcpq:
///  * Handshake:
///    * client -> server:  uint32 "magic number"
///    * client -> server:  uint32 "protocol version"
///    * client <- server:  uint32 "magic number"
///    * client <- server:  uint32 "server version"
///  * Messages (either direction, repeated forever)
///    * uint32 message type (1 = publish, 2 = subscribe, 3 = unsubscribe)
///    * uint32 channel_len
///    * char[channel_len] channel  (no null termination in-band)
///    * uint32 payload_len
///    * char[channel_len] payload  (ignored for subscribe/unsubscribe msgs)
///  * Shutdown upon TCP disconnect or invalid data.
///
/// All subscribe/unsubscribe messages are discarded for now.

/// Constructs a point-to-point LCM link over a UNIX socket.
///
/// Of the processes connecting, one must specify is_server=true, the other
/// false.  This is only for protocol details; there is no functional
/// difference between client and server in LCM.
std::unique_ptr<DrakeLcmInterface> LcmOverUnix(std::string socket_filename,
                                               bool is_server);

/// Constructs a point-to-point LCM link over a TCP connection.
///
/// Of the processes connecting, one must specify server=true, the other
/// server=false.  This is only for protocol details; there is no functional
/// difference between client and server in LCM.
std::unique_ptr<DrakeLcmInterface> LcmOverTcp(
    std::string server_address, int server_port,
    std::string client_address, int client_port,
    bool is_server);

}  // multiprocess
}  // drake
