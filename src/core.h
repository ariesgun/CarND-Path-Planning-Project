#ifndef CORE_H
#define CORE_H

#include <tuple>
#include <vector>

enum class CarState {
  KeepLane,
  PrepareLaneChange,
  LaneChange
};

// std::vector<struct sensor_data> sensor_data;
const double MAX_SPEED = 49.5;

class PathPlanner {
public:

  PathPlanner() : lane(1), speed(0.0), carState(CarState::KeepLane) {}

  /**
   * Generate the path plan.
   * */
  std::tuple<int, double> generate_plan(const double cur_speed,
    const double car_s,
    const double car_d,
    const double end_s,
    const int prev_size,
    const std::vector<std::vector<double>>& sensor_fusion) {

    double allowed_speed = MAX_SPEED;

    // std::array<std::vector<std::tuple<double, double>>, 3> lanes;
    std::array<std::vector<std::vector<double>>, 3> lanes;
    std::vector<double> blocked_lane;

    int cur_lane = ((car_d < (2 + 2)) && (car_d >= (0))) ? 0 : ((car_d < (8)) && (car_d >= (4))) ? 1 : 2;

    // Obtain surrounding cars information.
    for (int i = 0; i < sensor_fusion.size(); ++i) {
      float d = sensor_fusion[i][6];
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      int check_lane = (d / 4);

      // Check if there is any car in front of the ego car.
      if ((check_lane == cur_lane) && (check_car_s > car_s) && ((check_car_s - car_s) < 25.0)) {
        blocked_lane.emplace_back(check_speed);
        blocked_lane.emplace_back(check_car_s);
        std::cout << "[" << check_lane << "] There is a car in front lane. Distance: " << (check_car_s - car_s) << " with speed " << check_speed * 2.24 << std::endl;
      }

      // Predict s in the future
      check_car_s += ((double)prev_size * 0.02 * check_speed);

      // Check car in the front and behind.
      if (((check_car_s > end_s) && ((check_car_s - end_s) < 25.5)) || ((check_car_s < end_s) && (end_s - check_car_s < 6.0))) {
        lanes[check_lane].emplace_back(check_car_s);
        std::cout << "[" << check_lane << "] There is a car on the lane. Distance: " << (check_car_s - end_s) << " with speed " << check_speed * 2.24 << std::endl;
      }
    }

    // Adapt maximum speed
    if (!blocked_lane.empty()) {
      allowed_speed = std::min(MAX_SPEED, (blocked_lane[0] * 2.24 - (18 - (blocked_lane[1] - car_s)) * 0.5));
    }

    if (carState == CarState::KeepLane) {
      std::cout << "=Enter KeepLane state\n";
      // Accelerate till maximum speed
      if (speed < allowed_speed) {
        speed += 0.448;
      }
      else {
        if (std::abs(speed - allowed_speed) < 0.5) {
          speed = allowed_speed;
        }
        else {
          speed -= 0.896;
        }
      }

      // Check if it is possible to change lane.
      // Strategy: Wait until there is no car in the lane
      if (!blocked_lane.empty()) {
        if ((((lane == 0) || (lane == 2)) && lanes[1].empty()) ||
          ((lane == 1) && (lanes[0].empty() || lanes[2].empty()))) {
          carState = CarState::PrepareLaneChange;
        }
      }
    }
    else if (carState == CarState::PrepareLaneChange) {
      std::cout << "==Enter PrepareLaneChange state\n";

      if (speed < 41.0) {
        lane = find_next_lane(lane, car_s, sensor_fusion);
        std::cout << "** Switching to " << (lane == 0) ? "LEFT\n" : "RIGHT\n";
        carState = CarState::LaneChange;
      }
      else {
        speed -= 0.896;
      }
    }
    else if (carState == CarState::LaneChange) {
      std::cout << "===Enter LaneChange state\n";

      if (std::abs((2 + 4 * lane) - car_d) < 1.2) {
        carState = CarState::KeepLane;
      }
    }

    return std::make_tuple(lane, speed);

  }

private:

  double find_next_lane(int cur_lane, double cur_s, const std::vector<std::vector<double>>& sensor_fusion) {
    if ((cur_lane == 0) || (cur_lane == 2)) {
      return 1;
    }
    else if (cur_lane == 1) {

      int left_id = -1;
      int right_id = -1;
      int left_vel = 0;
      int right_vel = 0;

      for (int i = 0; i < sensor_fusion.size(); ++i) {
        float d = sensor_fusion[i][6];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx * vx + vy * vy);
        double check_car_s = sensor_fusion[i][5];

        int check_lane = (d / 4);

        if ((check_lane == 0) && (check_car_s > cur_s)) {
          if (left_id == -1) {
            left_id = i;
            left_vel = check_speed;
          }
          else {
            if (check_car_s < sensor_fusion[left_id][5]) {
              left_id = i;
              left_vel = check_speed;
            }
          }
        }
        if ((check_lane == 2) && (check_car_s > cur_s)) {
          if (right_id == -1) {
            right_id = i;
            right_vel = check_speed;
          }
          else {
            if (check_car_s < sensor_fusion[right_id][5]) {
              right_id = i;
              right_vel = check_speed;
            }
          }
        }
      }

      int left_score = left_id == -1 ? 0 : (left_vel / (sensor_fusion[left_id][5] - cur_s));
      int right_score = right_id == -1 ? 0 : (right_vel / (sensor_fusion[right_id][5] - cur_s));

      return left_score <= right_score ? 0 : 2;
    }
  }

  // Members
  int lane;
  double speed;
  CarState carState;
};

#endif