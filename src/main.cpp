#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;
 //unsigned long long obs_cnt = 0; // counter of the processed observations
  PID pid_s;
 // How the parameters were tuned (manually + twiddle):
  // steering: no twiddle -- P=0.2, I=0.004, D=3.0; throttle: fixed
  // steering: twiddle -- P=0.2, I=0.004, D=3.0; throttle: fixed
  // steering: twiddle -- P=0.41, I=0.02, D=8.0; throttle: fixed
  // steering: twiddle -- P=0.609, I=0.015, D=10.0; throttle: fixed
  // steering: twiddle -- P=0.61, I=0.0155, D=13.0; throttle: twiddle -- P=2.0, I=0.0, D=3.0
  // steering: twiddle -- P=0.3, I=0.015, D=13.0; throttle: twiddle -- P=2.5, I=0.0, D=6.0
  // steering: twiddle -- P=0.2, I=0.0165, D=7.0; throttle: twiddle P=5.0, I=0.0, D=8.0
  // steering: no twiddle -- P=0.15, I=0.0165, D=5.0; throttle: no twiddle -- P=7.0, I=0.0, D=0.5
  pid_s.Init(0.15, 0.0165, 5.0, 30, 0, 0.1, 0.005, 1.0, 0.2);
  PID pid_t;
  pid_t.Init(7.0, 0.0, 0.5, 30, 0, 0.5, 0.005, 1.0, 0.1);

  h.onMessage([&pid_s,&pid_t](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    unsigned long long obs_cnt = 0; // counter of the processed observations
  
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          
          pid_s.UpdateError(cte);
         double steering_angle = pid_s.Correction();
          
          // the steeper steering angle, the lesser throttle
          pid_t.UpdateError(std::max(fabs(steering_angle), fabs(cte)));
          double throttle       = 1.0 + pid_t.Correction();
          if (throttle > 1.0) {
            throttle = 1.0;
          } else if (throttle <= 0.0) {
            throttle = 0.3;
          }

          // observation processed
          ++obs_cnt;
          
          // DEBUG
          std::cout << "\n======[" << obs_cnt << "]======"
              << "\nCross Track Error:               " << cte
              << "\nCurrent Speed:                   " << speed
              << "\nCurrent Steering Angle:          " << angle
              << "\nPID Corrected Steering:          " << steering_angle
              << "\nPID Steering Coefficients:       " <<   "Kp = " << pid_s.Kp
                                                       << ", Ki = " << pid_s.Ki
                                                       << ", Kd = " << pid_s.Kd
              << "\nPID Corrected Throttle:          " << throttle
              << "\nPID Throttle Coefficients:       " <<   "Kp = " << pid_t.Kp
                                                       << ", Ki = " << pid_t.Ki
                                                       << ", Kd = " << pid_t.Kd
              << "\n======[" << obs_cnt << "]======\n"
              << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steering_angle;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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