/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/vils/proto/vils.pb.h"

#include "modules/common/util/message_util.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "cyber/time/clock.h"

using apollo::perception::TrafficLightDetection;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;
using apollo::vils::VilsKPI;
using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::common::util::FillHeader;

VilsKPI VilsKPI_msg_;
auto* traffic_ = VilsKPI_msg_.mutable_traffic_light();
auto* obstacles_ = VilsKPI_msg_.mutable_perception_obstacle();

void TrafficCallback(const std::shared_ptr<TrafficLightDetection>& msg) 
{
  //std::cout << "Traffic Callback" << std::endl;

  traffic_->Clear();
  // Assuming VilsKPI_msg_ has a repeated field for traffic_light
    for (const auto& traffic_light : msg->traffic_light()) {
        // Create a new TrafficLight message in VilsKPI_msg_
        auto* new_traffic_light = traffic_->Add();
        // Copy the color from msg to new_traffic_light
        new_traffic_light->set_color(
            static_cast<apollo::vils::TrafficLight_Color>(traffic_light.color()));

        // Copy other fields as needed
        new_traffic_light->set_id(traffic_light.id());
        new_traffic_light->set_confidence(traffic_light.confidence());
        new_traffic_light->set_tracking_time(traffic_light.tracking_time());
        new_traffic_light->set_blink(traffic_light.blink());
        new_traffic_light->set_remaining_time(traffic_light.remaining_time());
    }
}
void ObstacleCallback(const std::shared_ptr<PerceptionObstacles>& msg) 
{
   //std::cout << "Obstacle Callback" << std::endl;

    // Clear any existing obstacles in VilsKPI_msg_
    obstacles_->Clear();

    for (const auto& perception_obstacle : msg->perception_obstacle()) {
        // Create a new PerceptionObstacle message and add it to VilsKPI_msg_
        auto* new_obstacle = obstacles_->Add();

        // Copy the fields from msg to new_obstacle
        new_obstacle->set_id(perception_obstacle.id());
        new_obstacle->mutable_position()->CopyFrom(perception_obstacle.position());
        new_obstacle->set_theta(perception_obstacle.theta());
        new_obstacle->mutable_velocity()->CopyFrom(perception_obstacle.velocity());
        new_obstacle->set_length(perception_obstacle.length());
        new_obstacle->set_width(perception_obstacle.width());
        new_obstacle->set_height(perception_obstacle.height());

        // Copy other fields as needed...
    }
}

void PlanningCallback(const std::shared_ptr<ADCTrajectory>& msg) 
{
  // std::cout << "Planning Callback" << std::endl;
  VilsKPI_msg_.clear_road_max_velocity();
  
  VilsKPI_msg_.set_road_min_velocity(0);
  for(auto &l : msg->debug().planning_data().st_graph())
  {
    if(l.speed_limit_size() > 0)
      VilsKPI_msg_.set_road_max_velocity(l.speed_limit().at(0).v());
  }
}

int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // publisher
  auto talker_node = apollo::cyber::CreateNode("vils_node");
  auto talker = talker_node->CreateWriter<VilsKPI>("/apollo/vils/kpi");

  // subscribe
  auto traffic_reader_ = apollo::cyber::CreateNode("traffic_reader");  
  auto traffic_listener = 
      traffic_reader_->CreateReader<TrafficLightDetection>(
          "/apollo/perception/traffic_light", TrafficCallback);

  auto obstacle_reader_ = apollo::cyber::CreateNode("obstacle_reader");  
  auto obstacle_listener = 
      obstacle_reader_->CreateReader<PerceptionObstacles>(
          "/apollo/perception/obstacles", ObstacleCallback);

  auto planning_reader_ = apollo::cyber::CreateNode("planning_reader");  
  auto planning_listener = 
      planning_reader_->CreateReader<ADCTrajectory>(
          "/apollo/planning", PlanningCallback);
  
  Rate rate(50.0);
  while (apollo::cyber::OK()) {
    FillHeader("vils_kpi", &VilsKPI_msg_);
    talker->Write(VilsKPI_msg_);
    rate.Sleep();
  }
  apollo::cyber::WaitForShutdown();
  return 0;
}
