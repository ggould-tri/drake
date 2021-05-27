#pragma once

#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace multiprocess {

/// An implementation of `DrakeLcmInterface` that operates over a TCP
/// connection; it thus guarantees reliable in-order delivery at the cost
/// of losing all broadcast/multicast functionality.
class LcmOverTcp : public drake::lcm::DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmOverTcp);

  LcmOverTcp(

};

}  // multiprocess
}  // drake
