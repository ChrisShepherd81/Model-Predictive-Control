#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <uWS/uWS.h>
#include <cppad/cppad.hpp>

#include "Eigen-3.3/Eigen/Core"
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

#define MPH_TO_MS 0.44704
#define MS_TO_MPH 1/MPH_TO_MS
#define WRITE_OUTPUT 0
#define PRINT 0
#define PRINT_SPEED 1

//Number of predictions and dt used in FG_eval.hpp
size_t N = 12;
double dt = 0.2;

// The reference velocity is set to 40 mph.
double ref_v = 40*MPH_TO_MS;

//The weights for cost evaluation
CppAD::AD<double> cte_eval_weigth = 5.0;
CppAD::AD<double> delta_eval_weigth = 0.02494*ref_v-0.200000; //Workaround. -> Best delta cost has not been found yet.
CppAD::AD<double> delta_diff_eval_weigth = 1000.0;

// Checks if the SocketIO event has JSON data.
string hasData(string s);

int main(int argc, char* argv[]) {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  //Set values if parameters given via console parameters
  if(argc >= 5)
  {
    cte_eval_weigth = std::stod(argv[1]);
    delta_diff_eval_weigth = std::stod(argv[2]);
    ref_v = std::stod(argv[3])*MPH_TO_MS;
    delta_eval_weigth = 0.02494*ref_v-0.200000;
    N = std::stoi(argv[4]);
  }

  //Initialized on connect
  std::chrono::steady_clock::time_point last_timeStamp;

#if WRITE_OUTPUT
  //Construct filename with current date time
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream ss;
  ss << "test_" << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S") << ".csv";

  FileWriter fileWriter(ss.str(), 3, 6, N);
#endif

#if PRINT_SPEED
  double max_speed = 0;
  double avg_speed = 0;
  size_t counter = 0;
#endif

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
#if PRINT
    cout << sdata << endl;
#endif
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

          auto timeStamp = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(timeStamp - last_timeStamp);

          last_timeStamp = timeStamp;
          dt = elapsed.count() / 1000.0;

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double psi_unity = j[1]["psi_unity"];
          double v = ((double)j[1]["speed"])*MPH_TO_MS;

          //Create path and car coordinates
          Points car_path(ptsx, ptsy);
          Points car_position(px,py);

          //Translate car and path coordinates to 1st point in path
          car_path.translation(-ptsx[0], -ptsy[0]);
          car_position.translation(-ptsx[0], -ptsy[0]);

          //Rotate points by -psi
          car_path.rotation(-psi);
          car_position.rotation(-psi);

          //Create 3rd order polynomial of path
          Polynomial polynomial_car_path(car_path.getXVector(), car_path.getYVector(), 3);

          // The cross track error is calculated by evaluating the polynomial at x=rel_x subtracted by rel_y
          double car_rel_x = car_position[0][0];
          double car_rel_y = car_position[0][1];
          double cte = polynomial_car_path.polyeval(car_rel_x) - car_rel_y;

          //Calculate psi error.
          auto coeffs = polynomial_car_path.getCoefficients();
          double slope_at_rel_x = coeffs[1]+ (2*coeffs[2]* car_rel_x) + (3*coeffs[3]* std::pow(car_rel_x,2.0));
          double epsi = -std::atan(slope_at_rel_x);

          //Create state
          Eigen::VectorXd state(6);
          state << car_rel_x, car_rel_y, 0, v, cte, epsi;

          //Calculate steering angle and throttle from current state using MPC.
          auto vars = mpc.Solve(state, coeffs);

#if WRITE_OUTPUT
          //Write to file
          fileWriter.writeData(std::vector<double>(state.data(), state.data() + state.size()),
                               std::vector<double>(coeffs.data(), coeffs.data() + coeffs.size()),
                               car_path.getXStdVector(), car_path.getYStdVector(),
                               mpc.getPathX(), mpc.getPathY() );
#endif
#if PRINT
          std::cout << "x = " << vars[0] << std::endl;
          std::cout << "y = " << vars[1] << std::endl;
          std::cout << "psi = " << vars[2] << std::endl;
          std::cout << "v = " << vars[3] << std::endl;
          std::cout << "cte = " << vars[4] << std::endl;
          std::cout << "epsi = " << vars[5] << std::endl;
          std::cout << "delta = " << vars[6] << std::endl;
          std::cout << "a = " << vars[7] << std::endl;

          std::cout << "Cycle time: " << elapsed.count() << std::endl;
#endif

#if PRINT_SPEED
          //Calculate average speed
          double speed_mph =  v*MS_TO_MPH;
          counter++;
          avg_speed = ((avg_speed * (counter-1)) + speed_mph)/counter;

          //Remember maximum speed
          if(speed_mph > max_speed)
            max_speed = speed_mph;

          std::cout << "Avg. speed: " << avg_speed << " Max speed: " << max_speed << std::endl;
#endif
          json msgJson;

          //Set new actuations
          msgJson["steering_angle"] = -vars[6]/deg2rad(25);
          msgJson["throttle"] = vars[7];

          //Create mpc path in car coordinate system
          Points mpc_path(mpc.getPathX(), mpc.getPathY());

          //Transform path back in car space
          mpc_path.translation(-car_rel_x, -car_rel_y);

          //Display the MPC predicted trajectory
          msgJson["mpc_x"] = mpc_path.getXStdVector();
          msgJson["mpc_y"] = mpc_path.getYStdVector();

          //Transform path back in car space
          car_path.translation(-car_rel_x, -car_rel_y);

          //Display the waypoints/reference line
          msgJson["next_x"] = car_path.getXStdVector();
          msgJson["next_y"] = car_path.getYStdVector();

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([&h, &last_timeStamp](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    last_timeStamp = std::chrono::steady_clock::now();
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    std::cout << "Disconnected" << std::endl;
    exit(1);
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

string hasData(string s)
{
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
