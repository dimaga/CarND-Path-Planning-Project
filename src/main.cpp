#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "Map.h"
#include "Trajectory.h"
#include "Obstacles.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(const std::string& s) {
  const auto found_null = s.find("null");
  const auto b1 = s.find_first_of("[");
  const auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  const Map map{in_map_};
  Trajectory trajectory;
  Obstacles obstacles{map};

  int lane = 1;
  double ref_vel = 0.0;

  h.onMessage([&map,
               &trajectory,
               &obstacles,
               &ref_vel,
               &lane](uWS::WebSocket<uWS::SERVER> ws,
                      char *data,
                      size_t length,
                      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw_deg = j[1]["yaw"];
          const double car_speed_mph = j[1]["speed"];

          using std::vector;

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          const std::size_t prev_size = previous_path_x.size();

          const double future_car_s = prev_size > 0 ? end_path_s : car_s;

          obstacles.Clear();
          for (const auto& id_xy_vxy_sd : sensor_fusion) {
            const double x = id_xy_vxy_sd[1];
            const double y = id_xy_vxy_sd[2];
            const double vx = id_xy_vxy_sd[3];
            const double vy = id_xy_vxy_sd[4];

            obstacles.Add({x, y}, {vx, vy});
          }

          const double future_time_sec = prev_size * ITrajectory::kDtInSeconds;
          auto obstacle = obstacles.Forward(future_time_sec,
                                            future_car_s,
                                            lane,
                                            50);
          if (obstacle) {
            if (lane < 2) {
              auto right_obstacle = obstacles.Forward(future_time_sec,
                                                      future_car_s,
                                                      lane + 1,
                                                      50);
              if (right_obstacle) {
                ref_vel = obstacle->velocity_mph();
              } else {
                ref_vel = 45.0;
                lane = lane + 1;
              }
            } else if (lane > 0) {
              auto left_obstacle = obstacles.Forward(future_time_sec,
                                                     future_car_s,
                                                     lane - 1,
                                                     50);
              if (left_obstacle) {
                ref_vel = obstacle->velocity_mph();
              } else {
                ref_vel = 45.0;
                lane = lane - 1;
              }
            } else {
              ref_vel = obstacle->velocity_mph();
            }
          } else {
            ref_vel = 45.0;
          }

          double car_yaw_rad = deg2rad(car_yaw_deg);

          trajectory.set_car_pos({car_x, car_y});
          trajectory.set_car_yaw_rad(car_yaw_rad);
          trajectory.set_car_speed_mph(car_speed_mph);
          trajectory.set_previous_path(previous_path_x, previous_path_y);

          const auto xy0 = map.ToCartesian({car_s + 30, (2 + 4 * lane)});
          const auto xy1 = map.ToCartesian({car_s + 60, (2 + 4 * lane)});
          const auto xy2 = map.ToCartesian({car_s + 90, (2 + 4 * lane)});

          trajectory.Trace(ref_vel, {xy0, xy1, xy2});

          json msgJson;
          msgJson["next_x"] = trajectory.path_x();
          msgJson["next_y"] = trajectory.path_y();

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  const int port = 4567;

  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
