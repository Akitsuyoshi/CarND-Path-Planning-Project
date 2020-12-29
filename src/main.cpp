#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "cost.h"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

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

  // Start in lane 1
  int lane = 1;
  // Move a reference velocity to target
  double ref_vel = 0.;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &lane,
               &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data,
                         size_t length, uWS::OpCode opCode) {
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

          const int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // Possible next lanes
          vector<int> available_lanes;
          available_lanes.push_back(lane);

          if (lane == 0) {
            available_lanes.push_back(1);
          } else if (lane == 1) {
            available_lanes.push_back(0);
            available_lanes.push_back(2);
          } else if (lane == 2) {
            available_lanes.push_back(1);
          }

          bool too_close = false;

          for (int i = 0; i < sensor_fusion.size(); i++) {
            const double d = sensor_fusion[i][6];

            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
              const double vx = sensor_fusion[i][3];
              const double vy = sensor_fusion[i][4];
              const double check_speed = sqrt(vx * vx + vy * vy);

              double check_car_s = sensor_fusion[i][5];
              check_car_s += (double)prev_size * 0.02 * check_speed;
              if (check_car_s > car_s &&
                  (check_car_s - car_s) < BUFFER_BETWEEN_CAR) {
                too_close = true;
              }
            }
          }

          // Considers lane changes only when car ahead is too close
          if (too_close) {
            vector<float> costs;

            for (int i = 0; i < available_lanes.size(); i++) {
              const int available_lane = available_lanes[i];
              float cost = 0.;

              for (int i = 0; i < sensor_fusion.size(); i++) {
                const double d = sensor_fusion[i][6];

                if (d < (2 + 4 * available_lane + 2) &&
                    d > (2 + 4 * available_lane - 2)) {
                  const double vx = sensor_fusion[i][3];
                  const double vy = sensor_fusion[i][4];
                  const double check_speed = sqrt(vx * vx + vy * vy);

                  double check_car_s = sensor_fusion[i][5];
                  check_car_s += (double)prev_size * 0.02 * check_speed;
                  cost += calculate_cost(car_s, check_car_s);
                }
              }
              costs.push_back(cost);
            }

            // The most min cost tragectory to lane
            int best_lane;
            float min_cost = std::numeric_limits<float>::infinity();

            // Find best lane
            for (int i = 0; i < available_lanes.size(); i++) {
              int available_lane = available_lanes[i];
              float cost = costs[i];
              if (cost > min_cost) {
                continue;
              }

              min_cost = cost;
              best_lane = available_lane;
            }
            // If car cannot change lane, car decreases the velocity to prevent
            // collision
            if (best_lane == lane) {
              ref_vel -= .224;
            }
            // Set the most min cost lane
            lane = best_lane;
          } else if (ref_vel < 49.5) {
            ref_vel += .224;
          }

          double ref_x;
          double ref_y;
          double ref_x_prev;
          double ref_y_prev;
          double ref_yaw;

          if (prev_size == 0) {
            ref_x = car_x;
            ref_y = car_y;
            ref_x_prev = ref_x - cos(car_yaw);
            ref_y_prev = ref_y - sin(car_yaw);
            ref_yaw = deg2rad(car_yaw);
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            ref_x_prev = previous_path_x[prev_size - 2];
            ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }

          // Add evenly 30m points ahead of starting point in Frenet space
          vector<double> next_wp_0 =
              getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp_1 =
              getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp_2 =
              getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);

          // List of a widely spaced waypoints, evenly spaced at 30m
          vector<double> pts_x;
          vector<double> pts_y;

          // Add 5 points to x and y
          pts_x.insert(pts_x.end(), {ref_x_prev, ref_x, next_wp_0[0],
                                     next_wp_1[0], next_wp_2[0]});
          pts_y.insert(pts_y.end(), {ref_y_prev, ref_y, next_wp_0[1],
                                     next_wp_1[1], next_wp_2[1]});

          for (int i = 0; i < pts_x.size(); i++) {
            // Shift car reference angle to 0 degrees
            const double shift_x = pts_x[i] - ref_x;
            const double shift_y = pts_y[i] - ref_y;

            pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Spline
          tk::spline spl;
          spl.set_points(pts_x, pts_y);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          const double target_x = 30.;
          const double target_y = spl(target_x);
          const double target_dist = distance(target_x, target_y, 0, 0);
          const double N = target_dist / (0.02 * ref_vel / 2.24);

          double x_add_on = 0;

          for (int i = 0; i <= (50 - prev_size); i++) {
            double x_point = x_add_on + target_x / N;
            double y_point = spl(x_point);

            x_add_on = x_point;

            const double x_ref = x_point;
            const double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}