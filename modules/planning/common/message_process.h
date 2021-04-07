/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 */

#pragma once

#include <chrono>
#include <list>
#include <fstream>
#include <string>
#include <vector>
#include <utility>
#include <unordered_map>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/dreamview/proto/hmi_status.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/storytelling/proto/story.pb.h"

namespace apollo {
namespace planning {

class MessageProcess {
 public:
  bool Init(const PlanningConfig& planning_config);
  void Close();

  void OnChassis(const apollo::canbus::Chassis& chassis);

  void OnHMIStatus(apollo::dreamview::HMIStatus hmi_status);

  void OnLocalization(const apollo::localization::LocalizationEstimate& le);

  void OnPrediction(
      const apollo::prediction::PredictionObstacles& prediction_obstacles);

  void OnRoutingResponse(
      const apollo::routing::RoutingResponse& routing_response);

  void OnStoryTelling(const apollo::storytelling::Stories& stories);

  void OnTrafficLightDetection(
      const apollo::perception::TrafficLightDetection& traffic_light_detection);

  void ProcessOfflineData(const std::string &record_file);

 private:
  struct ADCCurrentInfo {
    std::pair<double, double> adc_cur_position_;
    std::pair<double, double> adc_cur_velocity_;
    std::pair<double, double> adc_cur_acc_;
    double adc_cur_heading_;
  };

    apollo::hdmap::LaneInfoConstPtr GetCurrentLane(
      const apollo::common::PointENU& position);
  bool GetADCCurrentRoutingIndex(int* road_index, double* road_s);

  int GetADCCurrentInfo(ADCCurrentInfo* adc_curr_info);

  void GenerateObstacleTrajectory(
      const int frame_num,
      const int obstacle_id,
      const ADCCurrentInfo& adc_curr_info,
      ObstacleFeature* obstacle_feature);

  void GenerateObstaclePrediction(
      const apollo::prediction::PredictionObstacle& prediction_obstacle,
      const ADCCurrentInfo& adc_curr_info,
      ObstacleFeature* obstacle_feature);

  void GenerateObstacleFeature(LearningDataFrame* learning_data_frame);

  bool GenerateLocalRoutingPassages(
      std::vector<std::vector<std::pair<std::string, double>>>*
          local_routing_passages);

  void GenerateRoutingFeature(LearningDataFrame* learning_data_frame);

  void GenerateTrafficLightDetectionFeature(
      LearningDataFrame* learning_data_frame);
  void GenerateADCTrajectoryPoints(
      const std::list<apollo::localization::LocalizationEstimate>&
          localizations,
      LearningDataFrame* learning_data_frame);

  void GeneratePlanningTag(LearningDataFrame* learning_data_frame);

  void GenerateLearningDataFrame(LearningDataFrame* learning_data_frame);

 private:
  PlanningConfig planning_config_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  std::ofstream log_file_;
  std::string record_file_;
  std::unordered_map<std::string, std::string> map_m_;
  LearningData learning_data_;
  int learning_data_file_index_ = 0;
  std::list<apollo::localization::LocalizationEstimate> localizations_;
  std::unordered_map<int, apollo::prediction::PredictionObstacle>
      prediction_obstacles_map_;
  std::unordered_map<int, std::list<PerceptionObstacleFeature>>
      obstacle_history_map_;
  ChassisFeature chassis_feature_;
  std::string map_name_;
  PlanningTag planning_tag_;
  apollo::routing::RoutingResponse routing_response_;
  double traffic_light_detection_message_timestamp_;
  std::vector<TrafficLightFeature> traffic_lights_;
  int total_learning_data_frame_num_ = 0;
};

}  // namespace planning
}  // namespace apollo
