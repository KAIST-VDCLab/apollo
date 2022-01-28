/* Copyright 2019 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/wey/wey_message_manager.h"

#include "modules/canbus/vehicle/wey/protocol/ads1_111.h"
#include "modules/canbus/vehicle/wey/protocol/ads3_38e.h"
#include "modules/canbus/vehicle/wey/protocol/ads_eps_113.h"
#include "modules/canbus/vehicle/wey/protocol/ads_req_vin_390.h"
#include "modules/canbus/vehicle/wey/protocol/ads_shifter_115.h"

#include "modules/canbus/vehicle/wey/protocol/fail_241.h"
#include "modules/canbus/vehicle/wey/protocol/fbs1_243.h"
#include "modules/canbus/vehicle/wey/protocol/fbs2_240.h"
#include "modules/canbus/vehicle/wey/protocol/fbs3_237.h"
#include "modules/canbus/vehicle/wey/protocol/fbs4_235.h"
#include "modules/canbus/vehicle/wey/protocol/status_310.h"
#include "modules/canbus/vehicle/wey/protocol/vin_resp1_391.h"
#include "modules/canbus/vehicle/wey/protocol/vin_resp2_392.h"
#include "modules/canbus/vehicle/wey/protocol/vin_resp3_393.h"

namespace apollo {
namespace canbus {
namespace wey {

WeyMessageManager::WeyMessageManager() {
  // Control Messages
  AddSendProtocolData<Ads1111, true>(); // acceleration
  AddSendProtocolData<Ads338e, true>(); // beam, horn, turning signal
  AddSendProtocolData<Adseps113, true>(); // steering
  AddSendProtocolData<Adsreqvin390, true>(); // ?
  AddSendProtocolData<Adsshifter115, true>(); // gear

  // Report Messages
  AddRecvProtocolData<Fail241, true>(); // fail safe hj
  AddRecvProtocolData<Fbs1243, true>(); // wheel dir type & spd. vehicle spd jy
  AddRecvProtocolData<Fbs2240, true>(); // wheel dir type & spd. vehicle spd jy
  AddRecvProtocolData<Fbs3237, true>(); // engine RPM, acc pedal pos, gear pos, steering whl torque not required
  AddRecvProtocolData<Fbs4235, true>(); // steering whl angle hj
  AddRecvProtocolData<Status310, true>(); // whl speed valid, steering whl pos, parking brake, beam, turning signal hj
  AddRecvProtocolData<Vinresp1391, true>();
  AddRecvProtocolData<Vinresp2392, true>();
  AddRecvProtocolData<Vinresp3393, true>();
}

WeyMessageManager::~WeyMessageManager() {}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
