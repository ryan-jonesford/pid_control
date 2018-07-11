#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "Inc/PID.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws) {
  // reset
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

// Twiddle param setup
float tolerance = 0.0001;
std::vector<double> Ds = {.001, .0001, .1};
std::vector<double>::iterator it_Ds = Ds.begin();
std::vector<double> taus;
enum {
  START = 0,
  CHECK_1,
  CHECK_2,
} state = START;
std::vector<double> best_v;

bool twiddle(PID& pid, double cnt, double& best,
             uWS::WebSocket<uWS::SERVER>& ws) {
  if ((Ds[0] + Ds[1] + Ds[2]) < tolerance) {
    return true;
  }
  size_t index = it_Ds - Ds.begin();
  bool loop = true;
  while (loop) {
    loop = false;
    bool inc_index = false;
    switch (state) {
      case START:
        pid.K_[index] += Ds[index];
        reset_simulator(ws);
        state = CHECK_1;
        break;
      case CHECK_1:
        if (cnt > best) {
          best_v = pid.K_;
          best = cnt;
          Ds[index] *= 1.1;
          inc_index = true;
          loop = true;
          state = START;
        } else {
          pid.K_[index] = pid.K_[index] - (2 * Ds[index]);
          reset_simulator(ws);
          state = CHECK_2;
        }
        break;
      case CHECK_2:
        if (cnt > best) {
          best_v = pid.K_;
          best = cnt;
          Ds[index] *= 1.1;
        } else {
          pid.K_[index] += Ds[index];
          Ds[index] *= .9;
        }
        inc_index = true;
        loop = true;
        state = START;
        break;
      default:
        state = START;
        it_Ds = Ds.begin();
        inc_index = false;
        loop = false;
        reset_simulator(ws);
    }
    if (inc_index) {
      if (++it_Ds == Ds.end()) {
        it_Ds = Ds.begin();
      }
    }
  }
  if ((Ds[0] + Ds[1] + Ds[2]) < tolerance) {
    return true;
  }
  return false;
}

int main() {
  uWS::Hub h;

  // twiddle param setup
  bool init_best = true;
  double best;
  int run_count = 0;
  bool optimized = false;
  PID pid(0.174175, 0.00014839, 9.03749);
  PID pid_spd(1000, 0.004, 50);
  float speed_sp = 60;
  int samples = 5000;

  h.onMessage([&pid, &pid_spd, &run_count, &speed_sp, &init_best, &best,
               &samples, &optimized](uWS::WebSocket<uWS::SERVER> ws, char* data,
                                     size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message
    // event. The 4 signifies a websocket message The 2 signifies a
    // websocket event
    double throttle = 0;
    double steer_value = 0;
    bool end_run = false;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          steer_value = pid.step(cte);
          steer_value = fabs(steer_value) > 1 ? steer_value / fabs(steer_value)
                                              : steer_value;
          throttle = pid_spd.step(speed - speed_sp);

          // DEBUG
          // std::cout << "CTE: " << cte <<" Steering Value: " <<
          // steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //   std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          if ( fabs(cte) > 1 && speed_sp > 15 ){
            speed_sp-=.75;
          }
          else if( speed_sp < 60 ){
            speed_sp+=.5;
          }
          // if (fabsf(cte) > 2) {  // Car is off track
          //   end_run = true;
          // }
          // // optimize for completing track
          // if ((run_count >= samples && !optimized) ||
          //     end_run && run_count > 10) {
          //   if (init_best) {
          //     best = run_count;
          //     best_v = pid.K_;
          //     init_best = false;
          //   }
          //   std::cout << "\n\nThis run = " << run_count << std::endl;
          //   std::cout << "kp = " << pid.K_[0] << " ki = " << pid.K_[1]
          //             << " kd = " << pid.K_[2] << std::endl;
          //   optimized = twiddle(pid, run_count, best, ws);
          //   std::cout << "best = " << best << std::endl;
          //   std::cout << "best kp = " << best_v[0] << " best ki = " << best_v[1]
          //             << " best kd = " << best_v[2] << std::endl;
          //   run_count = 0;
          //   end_run = false;
          // } else if (optimized) {
          //   std::cout << "*********************" << std::endl;
          //   std::cout << "         DONE" << std::endl;
          //   std::cout << "*********************" << std::endl;
          //   std::cout << "best kp = " << best_v[0] << " best ki = " << best_v[1]
          //             << " best kd = " << best_v[2] << std::endl;
          //   pid.K_ = best_v;
          //   if (fabsf(cte) > 2) {  // Car is off track
          //     reset_simulator(ws);
          //   }
          // } else {
          //   ++run_count;
          // }
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
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
                         char* message, size_t length) {
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
