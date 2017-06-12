#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "Points.h"
#include "Polynomial.h"
#include "FileWriter.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

#define MPH_TO_MS 0.44704
#define USE_GLOBAL_MAP_FOR_MPC 0
#define WRITE_OUTPUT 1

//Number of predictions declared in FG_eval.hpp
extern size_t N ;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  std::chrono::steady_clock::time_point last_timeStamp = std::chrono::steady_clock::now();

#if WRITE_OUTPUT
  //Construct filename with current date time
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream ss;
  ss << "test_" << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S") << ".csv";

  FileWriter fileWriter(ss.str(), 5, N);
#endif

  h.onMessage([&mpc,&last_timeStamp, &fileWriter](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

          auto timeStamp = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(timeStamp - last_timeStamp);
          std::cout << "Elapsed time: " << elapsed.count() << std::endl;
          last_timeStamp = timeStamp;

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double psi_unity = j[1]["psi_unity"];
          double v = ((double)j[1]["speed"])*MPH_TO_MS;

#if USE_GLOBAL_MAP_FOR_MPC
          //TODO: Doesn't work but why? Because of orientation? x -> -x
          //Create path in global coordinate system
          Points gloabl_path(ptsx, ptsy);

          //Polynomial of path in global coordinates
          Polynomial polynomial_global_path(gloabl_path.getXVector(), gloabl_path.getYVector(), 3);

          // The cross track error is calculated by evaluating the polynomial at x, f(x) and y subtracted.
          double cte = polynomial_global_path.polyeval(px) - py;

          //Calculate psi error.
          auto coeffs = polynomial_global_path.getCoefficients();
          double slope_at_px = coeffs[1] + (2*coeffs[2]*px) + (3*coeffs[3]*px*px);
          double atan = std::atan(slope_at_px);
          double epsi = -atan ;

          std::cout << "CTE: " << cte << " ePsi: " << epsi << " atan: " << atan << " slope: " << slope_at_px << std::endl;

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
#else
          //Create path and car coordinates
          Points car_path(ptsx, ptsy);
          Points car_position(px,py);

          //Translate car and path coordinates to 1st point in path
          car_path.translation(-ptsx[0], -ptsy[0]);
          car_position.translation(-ptsx[0], -ptsy[0]);

          //Rotate points by -psi
          car_path.rotation(-psi);
          car_position.rotation(-psi);

          //3rd order polynomial of path
          Polynomial polynomial_car_path(car_path.getXVector(), car_path.getYVector(), 3);

          // The cross track error is calculated by evaluating the polynomial at x=rel_x subtracted by rel_y
          double car_rel_x = car_position[0][0];
          double car_rel_y = car_position[0][1];
          double cte = polynomial_car_path.polyeval(car_rel_x) - car_rel_y;

          //Calculate psi error.
          auto coeffs = polynomial_car_path.getCoefficients();
          double slope_at_rel_x = coeffs[1]+ (2*coeffs[2]* car_rel_x) + (3*coeffs[3]* std::pow(car_rel_x,2.0));
          double epsi = -std::atan(slope_at_rel_x);

          //std::cout << "CTE: " << cte << " ePsi: " << epsi << " atan: " << atan << " slope: " << slope_at_0 << std::endl;

          Eigen::VectorXd state(6);
          state << car_rel_x, car_rel_y, 0, v, cte, epsi;
          std::cout << "test\n";
#endif

          //Calculate steering angle and throttle using MPC.
          auto vars = mpc.Solve(state, coeffs);

#if WRITE_OUTPUT
          fileWriter.writeData(std::vector<double>(state.data(), state.data() + state.size()),
                               car_path.getXStdVector(), car_path.getYStdVector(),
                               mpc.getPathX(), mpc.getPathY() );
#endif

          state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];

#if 1
          std::cout << "x = " << vars[0] << std::endl;
          std::cout << "y = " << vars[1] << std::endl;
          std::cout << "psi = " << vars[2] << std::endl;
          std::cout << "v = " << vars[3] << std::endl;
          std::cout << "cte = " << vars[4] << std::endl;
          std::cout << "epsi = " << vars[5] << std::endl;
          std::cout << "delta = " << vars[6] << std::endl;
          std::cout << "a = " << vars[7] << std::endl;
#endif

          double steer_value = vars[6];
          double throttle_value = vars[7];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].

          msgJson["steering_angle"] = -steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

#if USE_GLOBAL_MAP_FOR_MPC
          //Create mpc path in car coordinate system
          Points mpc_path(mpc.getPathX(), mpc.getPathY());
          mpc_path.translation(-px, -py);
          mpc_path.rotation(-psi);

          msgJson["mpc_x"] = mpc_path.getXStdVector();
          msgJson["mpc_y"] = mpc_path.getYStdVector();
#else

          //Create mpc path in car coordinate system
          Points mpc_path(mpc.getPathX(), mpc.getPathY());

          //Transform pathes back in car space
          mpc_path.translation(-car_rel_x, -car_rel_y);
          car_path.translation(-car_rel_x, -car_rel_y);

          msgJson["mpc_x"] = mpc_path.getXStdVector();
          msgJson["mpc_y"] = mpc_path.getYStdVector();
#endif

          //Display the waypoints/reference line
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

#if USE_GLOBAL_MAP_FOR_MPC
          //Create path in car coordinate system
          Points car_path = gloabl_path;
          car_path.translation(-px, -py);
          car_path.rotation(-psi);
#endif
          msgJson["next_x"] = car_path.getXStdVector();
          msgJson["next_y"] = car_path.getYStdVector();


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          //exit(1);
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

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
