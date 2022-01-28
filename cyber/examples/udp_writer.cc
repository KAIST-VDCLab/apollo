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
#define TEST 0
#define PRINT 0
#include "cyber/examples/proto/examples.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "cyber/cyber.h"
#include <boost/asio.hpp>
#include "DetectionData.h"
#include "ChatterData.h"

using namespace boost::asio;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;


google::protobuf::RepeatedPtrField<PerceptionObstacle> object_array;

void MessageCallback(const std::shared_ptr<PerceptionObstacles>& msg) {
  
  object_array.empty();
  object_array = msg->perception_obstacle();
    
  // sort by longitudinal distance
  std::sort(object_array.begin(), object_array.end(),
          [](const PerceptionObstacle& a, const PerceptionObstacle& b){
             return a.position().x() < b.position().x();
          });
#if PRINT
  for(const PerceptionObstacle& obs : object_array){
    AINFO << '\n' <<
      "obstacle id: " << obs.id() << '\n' <<
      "object type: " << obs.type() << '\n' <<
      "object subtype: " << obs.sub_type() << '\n' <<
      "object vel x: " << obs.velocity().x() << '\n' <<
      "object vel y: " << obs.velocity().y() << '\n' <<
      "object length: " << obs.length() << '\n' <<
      "object width: " << obs.width() << '\n' <<
      "object vel cov: \n[ " << obs.velocity_covariance()[0] << ' ' << obs.velocity_covariance()[1] << ' ' << obs.velocity_covariance()[2] << " ]" << '\n' <<
      "[ " << obs.velocity_covariance()[3] << ' ' << obs.velocity_covariance()[4] << ' ' << obs.velocity_covariance()[5] << " ]" << '\n' <<
      "[ " << obs.velocity_covariance()[6] << ' ' << obs.velocity_covariance()[7] << ' ' << obs.velocity_covariance()[8] << " ]" << '\n';
  }
#endif

  io_service io_service;
  ip::udp::endpoint local_endpoint;
  local_endpoint = ip::udp::endpoint(ip::address::from_string("192.168.0.99"), 4001); 
  // local_endpoint = ip::udp::endpoint(ip::address::from_string("192.168.0.60"), 5001); // original
  ip::udp::socket socket(io_service, local_endpoint); 
  ip::udp::endpoint remote_endpoint;
  
  
  boost::system::error_code err;
  // socket.open(ip::udp::v4());
  
  remote_endpoint = ip::udp::endpoint(ip::address::from_string("192.168.0.76"), 4001); // autobox
  // remote_endpoint = ip::udp::endpoint(ip::address::from_string("192.168.0.77"), 5001); // laptop
  


  uint32_t object_data[20*18];
  uint8_t object_data_send[20*18*4] ={0,};

  /***********************\ 
  TODO: sort by importance 
  \***********************/

  int i = 0;

#if TEST
  while(i<20){
    // 1: Object ID
    object_data[18*i + 0] = uint32_t(i*10000);
    // 2: Flags 
    object_data[18*i + 1] = uint32_t(0);
    // 3: Classification
    object_data[18*i + 2] = uint32_t(3*10000);
    // 4: Classification age
    object_data[18*i + 3] = uint32_t(0);
    // 5-6: Object box center(x,y)
    object_data[18*i + 4] = uint32_t(243.59*10000);
    object_data[18*i + 5] = uint32_t(40.78*10000);
    // 7: Object course angl 
    object_data[18*i + 6] = uint32_t(45.1234*10000);
    // 8: Object course angle Sigma, 
    object_data[18*i + 7] = uint32_t(0);
    // 9-10: Relative velocity(x,y) 
    object_data[18*i + 8] = uint32_t(43.78*10000);
    object_data[18*i + 9] = uint32_t(8.28*10000);
    // 11-12: Relative velocity Sigma(x,y)
    object_data[18*i + 10] = uint32_t(0);
    object_data[18*i + 11] = uint32_t(0);
    // 13-14: Absolute velocity(x,y) 
    object_data[18*i + 12] = uint32_t(0);
    object_data[18*i + 13] = uint32_t(0);
    // 15-16: Absolute velocity Sigma(x,y)
    object_data[18*i + 14] = uint32_t(0);
    object_data[18*i + 15] = uint32_t(0);
    // 17-18: Object box size(x,y)
    object_data[18*i + 16] = uint32_t(2.5468 * 10000);
    object_data[18*i + 17] = uint32_t(1.4658 * 10000);
    i++;
  }

#else
  // Ibeo: x=forward, y=left, Apollo: x=forward, y=left
  // AINFO << "instances after: ";
  for (const auto &object : object_array) {
  
  // remove behind object & cut 20 objects by distance
    if(i==20) continue;
    if(object.position().x() < 0) continue;
     
    //AINFO << '(' << object.position().x() << ", " << object.position().y() << ')';
  // 1: Object ID
    object_data[18*i + 0] = uint32_t(object.id()*10000);

  // 2: Flags 
    object_data[18*i + 1] = uint32_t(0);

  // 3: Classification
    uint32_t classification = 0;
    if (object.type() == 0 || object.type() == 1 || object.type() == 2) {
      // if unknown/unknown_movable/unknown_unmovable
      classification = (object.length() * object.width() < 25) ? 1 : 2; // unknown_small/unknown_big
    }
    else if(object.type() == 5) { // vehicle
      classification = (object.sub_type()==5 || object.sub_type()==6)?6:5; // if truck/bus -> 6, else -> 5
    }
    else {
      classification = uint32_t(object.type());
    }
    object_data[18*i + 2] = classification*10000;
    
    // AINFO << "object.type: " << object.type() << '\n' << "classification: " << classification;

  // 4: Classification age
    object_data[18*i + 3] = uint32_t(object.tracking_time()*10000);

  // 5-6: Object box center(x,y)
    object_data[18*i + 4] = uint32_t(object.position().x()*10000);
    object_data[18*i + 5] = uint32_t(object.position().y()*10000);

  // 7: Object course angl 
    object_data[18*i + 6] = uint32_t(object.theta()*10000);

  // 8: Object course angle Sigma, 
    object_data[18*i + 7] = uint32_t(0);

  // 9-10: Relative velocity(x,y) 
    object_data[18*i + 8] = uint32_t(object.velocity().x()*10000);
    object_data[18*i + 9] = uint32_t(object.velocity().y()*10000);

  // 11-12: Relative velocity Sigma(x,y)
    object_data[18*i + 10] = uint32_t(sqrt(object.velocity_covariance()[0])*10000);
    object_data[18*i + 11] = uint32_t(sqrt(object.velocity_covariance()[4])*10000);

  // 13-14: Absolute velocity(x,y)
    object_data[18*i + 12] = uint32_t(0);
    object_data[18*i + 13] = uint32_t(0);

  // 15-16: Absolute velocity Sigma(x,y)
    object_data[18*i + 14] = uint32_t(0);
    object_data[18*i + 15] = uint32_t(0);

  // 17-18: Object box size(x,y)
    object_data[18*i + 16] = uint32_t(object.length() * 10000);
    object_data[18*i + 17] = uint32_t(object.width() * 10000);

    i++;
  }
#endif
  memcpy(object_data_send,object_data,20*18*4);

  socket.send_to(buffer(&object_data_send[0], 20*18*4), remote_endpoint, 0, err);
  socket.close();
}


int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("udp_writer");
  // create listener
  
  auto listener = listener_node->CreateReader<PerceptionObstacles>("/apollo/perception/obstacles", MessageCallback);
  
  apollo::cyber::WaitForShutdown();
  return 0;
}
