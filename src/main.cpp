#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "core.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  PathPlanner pathPlanner;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
    &map_waypoints_dx, &map_waypoints_dy, &pathPlanner]
    (uWS::WebSocket<uWS::SERVER>* ws, char* data, size_t length,
      uWS::OpCode opCode) {

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

          auto s = hasData(data);

          if (s != "") {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry") {
              // j[1] is the data JSON object

              // Main car's localization Data
              double car_x = j[1]["x"];
              double car_y = j[1]["y"];
              double car_s = j[1]["s"];
              double car_d = j[1]["d"];
              double car_yaw = j[1]["yaw"];
              double car_speed = j[1]["speed"];

              // Previous path data given to the Planner
              auto previous_path_x = j[1]["previous_path_x"];
              auto previous_path_y = j[1]["previous_path_y"];
              // Previous path's end s and d values 
              double end_path_s = j[1]["end_path_s"];
              double end_path_d = j[1]["end_path_d"];

              // Sensor Fusion Data, a list of all other cars on the same side 
              //   of the road.
              auto sensor_fusion = j[1]["sensor_fusion"];

              json msgJson;

              vector<double> next_x_vals;
              vector<double> next_y_vals;

              /**
               * Define a path made up of (x,y) points that the car will visit
               *   sequentially every .02 seconds
               */
              int prev_size = previous_path_x.size();
              if (prev_size <= 0) {
                // car_s = end_path_s;
                end_path_s = car_s;
              }

              std::tuple<int, double> planInfo = pathPlanner.generate_plan(car_speed,
                car_s,
                car_d,
                end_path_s,
                prev_size,
                sensor_fusion);
              int lane = std::get<0>(planInfo);
              double ref_vel = std::get<1>(planInfo);

              /*
               * Trajectory Generation starts here!.
               */
               // Create a list of widely space (x,y) waypoints. Spline will interpolate these waypoints
              vector<double> ptsx;
              vector<double> ptsy;

              // Reference x,y, yaw
              double ref_x = car_x;
              double ref_y = car_y;
              double ref_yaw = deg2rad(car_yaw);

              // If previous size is empty, use the car as the starting reference
              if (prev_size < 2) {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
              }
              else {
                //Redefine reference staate as previous path and point
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_prev_x = previous_path_x[prev_size - 2];
                double ref_prev_y = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);

                ptsx.push_back(ref_prev_x);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_prev_y);
                ptsy.push_back(ref_y);
              }

              // In Frenet, add evenly 30m spaced points ahead of the starting reference.
              vector<double> next_wp_0 = getXY(car_s + 45, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> next_wp_1 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> next_wp_2 = getXY(car_s + 135, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

              ptsx.push_back(next_wp_0[0]);
              ptsx.push_back(next_wp_1[0]);
              ptsx.push_back(next_wp_2[0]);

              ptsy.push_back(next_wp_0[1]);
              ptsy.push_back(next_wp_1[1]);
              ptsy.push_back(next_wp_2[1]);

              for (int i = 0; i < ptsx.size(); i++) {
                // Shift car reference angle to 0 degree (car's coordinate / point of view)

                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;

                ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
              }

              tk::spline s;
              s.set_points(ptsx, ptsy);

              for (int i = 0; i < previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

              // Calculate how to break up spline points so that we travel at our desired reference velocity
              double target_x = 40.0;
              double target_y = s(target_x);
              double target_dist = sqrt((target_x) * (target_x)+(target_y) * (target_y));

              double x_add_on = 0;

              for (int i = 0; i <= 50 - previous_path_x.size(); i++) {
                double N = (target_dist / (0.02 * ref_vel / 2.24));
                double x_point = x_add_on + target_x / N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                // Rotate back to normal after rotating it earlier;
                x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
              }

              /**
               * End TODO
               */

              msgJson["next_x"] = next_x_vals;
              msgJson["next_y"] = next_y_vals;

              auto msg = "42[\"control\"," + msgJson.dump() + "]";



              ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }  // end "telemetry" if
          }
          else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }  // end websocket if
    }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code,
    char* message, size_t length) {
      ws->close();
      std::cout << "Disconnected" << std::endl;
    });

  int port = 4567;
  if (h.listen("127.0.0.1", port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}