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
#include "spline.h"

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
  Map map(in_map_);

  int lane = 1;
  float ref_vel = 0.0;

  h.onMessage([&map, &ref_vel, lane](uWS::WebSocket<uWS::SERVER> ws,
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
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

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

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;

          for (const auto& id_xy_vxy_sd : sensor_fusion) {
            const int id = id_xy_vxy_sd[0];
            const double x = id_xy_vxy_sd[1];
            const double y = id_xy_vxy_sd[2];
            const double vx = id_xy_vxy_sd[3];
            const double vy = id_xy_vxy_sd[4];
            const double s = id_xy_vxy_sd[5];
            const double d = id_xy_vxy_sd[6];

            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
              const double check_speed = std::sqrt(vx*vx + vy*vy);

              const double check_car_s = s + ((double)prev_size*.02*check_speed);
              if (check_car_s > car_s &&check_car_s - car_s < 30) {
                too_close = true;
                break;
              }
            }

            // std::cout << "id = " << id << "\n";
            // std::cout << "x = " << x << "\n";
            // std::cout << "y = " << y << "\n";
            // std::cout << "vx = " << vx << "\n";
            // std::cout << "vy = " << vy << "\n";
            // std::cout << "s = " << s << "\n";
            // std::cout << "d = " << d << "\n\n";
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2) {
            const double prev_car_x = car_x - std::cos(ref_yaw);
            const double prev_car_y = car_y - std::sin(ref_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            ref_x = previous_path_x.at(prev_size - 1);
            ref_y = previous_path_y.at(prev_size - 1);

            const double ref_x_prev = previous_path_x.at(prev_size - 2);
            const double ref_y_prev = previous_path_y.at(prev_size - 2);
            ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          const auto xy0 = map.ToCartesian({car_s + 30, (2 + 4 * lane)});
          const auto xy1 = map.ToCartesian({car_s + 60, (2 + 4 * lane)});
          const auto xy2 = map.ToCartesian({car_s + 90, (2 + 4 * lane)});

          ptsx.push_back(xy0.x());
          ptsx.push_back(xy1.x());
          ptsx.push_back(xy2.x());

          ptsy.push_back(xy0.y());
          ptsy.push_back(xy1.y());
          ptsy.push_back(xy2.y());

          const double cos_neg_yaw = std::cos(-ref_yaw);
          const double sin_neg_yaw = std::sin(-ref_yaw);

          // Convert in local reference frame of the car
          for (std::size_t i = 0, sz = ptsx.size(); i < sz; ++i) {
            const double shift_x = ptsx[i] - ref_x;
            const double shift_y = ptsy[i] - ref_y;

            ptsx.at(i) = shift_x * cos_neg_yaw - shift_y * sin_neg_yaw;
            ptsy.at(i) = shift_x * sin_neg_yaw + shift_y * cos_neg_yaw;
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals = previous_path_x;
          vector<double> next_y_vals = previous_path_y;

          const double target_x = 30.0;
          const double target_y = s(target_x);
          const double target_dist = Eigen::Vector2d{target_x, target_y}.norm();

          double x_add_on = 0;

          const double cos_yaw = std::cos(ref_yaw);
          const double sin_yaw = std::sin(ref_yaw);

          for (std::size_t i = previous_path_x.size(); i < 50; ++i) {
            const double N = target_dist / (.02 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            const double x_ref = x_point;
            const double y_ref = y_point;

            x_point = x_ref * cos_yaw - y_ref * sin_yaw;
            y_point = x_ref * sin_yaw + y_ref * cos_yaw;

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

            if (too_close && ref_vel > 0.224) {
              ref_vel -= 0.224;
            } else if(ref_vel < 49.5) {
              ref_vel += 0.224;
            }
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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
