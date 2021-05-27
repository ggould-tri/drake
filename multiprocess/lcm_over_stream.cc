#include "drake/multiprocess/lcm_over_stream.h"

namespace drake {
namespace multiprocess {

namespace {

constexpr static uint32_t kServerMagicNumber 0x287617fa;
constexpr static uint32_t kMagicClient 0x287617fb;
constexpr static uint32_t kProtocolVersion 0x0100;

enum class MessageType {
  kPublish = 1,
  kSubscribe = 2,
  kUnsubscribe = 3,
};

/// Receive exactly @p len bytes from @p fd into @p buffer, blocking if
/// necessary.  Returns `nullptr` on error.
std::unique_ptr<char[]> ReadExactly(int fd, int len) {
  std::unique_ptr<char[]> result(new char[len]);
  const int received = recv(fd, *result, len, MSG_WAITALL);
  if (received == len) {
    return result;
  } else {
    return nullptr;
  }
}

/// Read a single network-byte-order uint32 from a file descriptor, blocking
/// if necessary; return that value or std::nullopt on error.
std::optional<uint32_t> ReadUint32(int fd) {
  char raw_data[4];
  const int received = recv(fd, raw_data, 4, MSG_WAITALL);
  if (received == 4) {
    return ntohol(static_cast<uint32_t>(*raw_data));
  } else {
    return std::nullopt;
  }
}

/// Send exactly and fully the contents of @p data on @p fd, blocking if
/// necessary; return `true` if successful, `false` if unsuccessful.
bool SendExactly(int fd, const std::vector<char>& data) {
  const int sent = send(fd, data.data, data.size(), 0);
  return (sent == data.size());
}

/// Send @p data over @p fd, blocking if necessary; return `true` if
/// successful, `false` if unsuccessful.
bool SendUint32(int fd, uint32_t data) {
  const uint32 network_order_data = htonl(data);
  const char* raw_data = static_cast<const char*>(&network_order_data);
  const int sent = send(fd, raw_data, 4, 0);
  return (sent = 4);
}

}  // namespace

std::unique_ptr<DrakeLcmInterface> LcmOverUnix(std::string socket_filename,
                                               bool is_server) {

}

std::unique_ptr<DrakeLcmInterface> LcmOverTcp(
    std::string server_address, int server_port,
    std::string client_address, int client_port,
    bool is_server) {

}

}  // multiprocess
}  // drake
