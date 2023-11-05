/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/localization/rtk/rtk_localization_component.h"
#include "cyber/time/clock.h"
#include "cyber/time/rate.h"

using apollo::cyber::Rate;
Rate rate(50.0);

bool TIME_FLAG = true;
double time_reference = 0.0;
double localization_timestamp = 0.0;

bool Me_TIME_FLAG = true;
double mes_time_reference =0.0;
double localization_mes_timestamp = 0.0;

namespace apollo {
namespace localization {

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, 80, "/tmp/localization_log_%F_%H%M%S.csv",
           &time_tm);
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {
  file_stream << "time,"
              << "mes_time,"
              << "localization_x,"
              << "localization_y,"
              << "localization_z,"
              << "heading,"
              << "angular_velocity_x,"
              << "angular_velocity_y,"
              << "angular_velocity_z," << std::endl;
}
}  // namespace

void RTKLocalizationComponent::ProcessLogs(const LocalizationEstimate &localization) {
  const std::string log_str = absl::StrCat(
    localization_timestamp, ",",
    localization_mes_timestamp, ",",
    localization.pose().position().x() - 302590, ",",
    localization.pose().position().y() - 4124220, ",",
    localization.pose().position().z(), ",",
    localization.pose().heading(), ",",
    localization.pose().angular_velocity().x(), ",",
    localization.pose().angular_velocity().y(), ",",
    localization.pose().angular_velocity().z()
  );
  if (1) {
    localization_log_file_ << log_str << std::endl;
  }
}

RTKLocalizationComponent::RTKLocalizationComponent()
    : localization_(new RTKLocalization()) 
    {
      if(1){
        localization_log_file_.open(GetLogFileName());
        localization_log_file_ << std::fixed;
        localization_log_file_ << std::setprecision(10);
        WriteHeaders(localization_log_file_);
      }
    }
RTKLocalizationComponent::~RTKLocalizationComponent()
{
  CloseLogFile();
}
void RTKLocalizationComponent::CloseLogFile() {
  if (1 && localization_log_file_.is_open()) {
    localization_log_file_.close();
  }
}
bool RTKLocalizationComponent::Init() {
  tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(node_));
  if (!InitConfig()) {
    AERROR << "Init Config falseed.";
    return false;
  }

  if (!InitIO()) {
    AERROR << "Init Interval falseed.";
    return false;
  }

  return true;
}

bool RTKLocalizationComponent::InitConfig() {
  rtk_config::Config rtk_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &rtk_config)) {
    return false;
  }
  AINFO << "Rtk localization config: " << rtk_config.DebugString();

  localization_topic_ = rtk_config.localization_topic();
  localization_status_topic_ = rtk_config.localization_status_topic();
  imu_topic_ = rtk_config.imu_topic();
  gps_topic_ = rtk_config.gps_topic();
  gps_status_topic_ = rtk_config.gps_status_topic();
  broadcast_tf_frame_id_ = rtk_config.broadcast_tf_frame_id();
  broadcast_tf_child_frame_id_ = rtk_config.broadcast_tf_child_frame_id();

  localization_->InitConfig(rtk_config);

  return true;
}

bool RTKLocalizationComponent::InitIO() {
  corrected_imu_listener_ = node_->CreateReader<localization::CorrectedImu>(
      imu_topic_, std::bind(&RTKLocalization::ImuCallback, localization_.get(),
                            std::placeholders::_1));
  ACHECK(corrected_imu_listener_);

  gps_status_listener_ = node_->CreateReader<drivers::gnss::InsStat>(
      gps_status_topic_, std::bind(&RTKLocalization::GpsStatusCallback,
                                   localization_.get(), std::placeholders::_1));
  ACHECK(gps_status_listener_);

  localization_talker_ =
      node_->CreateWriter<LocalizationEstimate>(localization_topic_);
  ACHECK(localization_talker_);

  localization_status_talker_ =
      node_->CreateWriter<LocalizationStatus>(localization_status_topic_);
  ACHECK(localization_status_talker_);
  return true;
}

bool RTKLocalizationComponent::Proc(
    const std::shared_ptr<localization::Gps>& gps_msg) {
  localization_->GpsCallback(gps_msg);

  if (localization_->IsServiceStarted()) {
    LocalizationEstimate localization;
    localization_->GetLocalization(&localization);
    LocalizationStatus localization_status;
    localization_->GetLocalizationStatus(&localization_status);

    // publish localization messages
    PublishPoseBroadcastTopic(localization);
    PublishPoseBroadcastTF(localization);
    PublishLocalizationStatus(localization_status);
    // rate.Sleep();
    ADEBUG << "[OnTimer]: Localization message publish success!";
  }

  return true;
}

void RTKLocalizationComponent::PublishPoseBroadcastTF(
    const LocalizationEstimate& localization) {
  // broadcast tf message
  apollo::transform::TransformStamped tf2_msg;

  auto mutable_head = tf2_msg.mutable_header();
  mutable_head->set_timestamp_sec(localization.measurement_time());
  mutable_head->set_frame_id(broadcast_tf_frame_id_);
  tf2_msg.set_child_frame_id(broadcast_tf_child_frame_id_);

  auto mutable_translation = tf2_msg.mutable_transform()->mutable_translation();
  mutable_translation->set_x(localization.pose().position().x());
  mutable_translation->set_y(localization.pose().position().y());
  mutable_translation->set_z(localization.pose().position().z());

  auto mutable_rotation = tf2_msg.mutable_transform()->mutable_rotation();
  mutable_rotation->set_qx(localization.pose().orientation().qx());
  mutable_rotation->set_qy(localization.pose().orientation().qy());
  mutable_rotation->set_qz(localization.pose().orientation().qz());
  mutable_rotation->set_qw(localization.pose().orientation().qw());

  tf2_broadcaster_->SendTransform(tf2_msg);
}

void RTKLocalizationComponent::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  // std::cout << localization.DebugString() << std::endl;
    if (TIME_FLAG){
    time_reference = localization.header().timestamp_sec();
    TIME_FLAG = false;
  }

    if (Me_TIME_FLAG){
    mes_time_reference = localization.measurement_time();
    Me_TIME_FLAG = false;
  }
  localization_timestamp = localization.header().timestamp_sec() - time_reference;
  localization_mes_timestamp = localization.measurement_time() - mes_time_reference;
  ProcessLogs(localization);
  localization_talker_->Write(localization);
}

void RTKLocalizationComponent::PublishLocalizationStatus(
    const LocalizationStatus& localization_status) {
  localization_status_talker_->Write(localization_status);
}

}  // namespace localization
}  // namespace apollo
