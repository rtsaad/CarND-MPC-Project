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
#include "matplotlibcpp.h"


//Plot CTE, Delta and Speed
//Requires matplotlib
bool plotting = false;
int max_iters = 100;
namespace plt = matplotlibcpp;

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  //For plotting
  int iters = 0;
  std::vector<double> x_vals = {};
  std::vector<double> y_vals = {};
  std::vector<double> psi_vals = {};
  std::vector<double> v_vals = {};
  std::vector<double> cte_vals = {};
  std::vector<double> epsi_vals = {};
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};  

  h.onMessage([&mpc, &iters, &x_vals, &y_vals, &psi_vals, &v_vals, &cte_vals, &epsi_vals, &delta_vals, &a_vals](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];	  
          double v = j[1]["speed"];
	  double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];	  

	  //Adjust plain path to car coordinates
	  //Set x, y and psi to zero
      	  for(unsigned int i=0; i < ptsx.size(); i++){
      
      	    //shift car reference angle to 90 degrees
      	    double shift_x = ptsx[i] -px;
      	    double shift_y = ptsy[i] -py;
      
      	    ptsx[i] = (shift_x * cos(0-psi) - shift_y*sin(0-psi));
      	    ptsy[i] = (shift_x * sin(0-psi) + shift_y*cos(0-psi));
      	    
      	  }
      
      	  double* ptrx = &ptsx[0];
      	  Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
      	  
      	  double* ptry = &ptsy[0];
      	  Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);
	  //Fit the Polynomial
      	  auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
      
	  //Get cte and epsi
      	  double cte = polyeval(coeffs, 0);
      	  double epsi = -atan(coeffs[1]);

	  //Plotting
	  if(plotting){
	    //If set, save data for plotting
	    iters++;
	    x_vals.push_back(px);
	    y_vals.push_back(py);
	    psi_vals.push_back(psi);
	    v_vals.push_back(v);
	    cte_vals.push_back(cte);
	    epsi_vals.push_back(epsi);	  
	    delta_vals.push_back(steer_value);
	    a_vals.push_back(throttle_value);
	  }

	  //predict next state to avoid Latency problems
	  //Latency of .1 seconds
	  double dt = 0.1;
	  const double Lf = 2.67;
	  double x1=0, y1=0,  psi1=0, v1=v, cte1=cte, epsi1=epsi;	  
	  x1 += v * cos(0) * dt;
	  y1 += v * sin(0) * dt;
	  //steer_value is negative
	  psi1 += - v/Lf * steer_value * dt;
	  v1 += throttle_value * dt;	    
	  cte1 +=   v * sin(epsi1) * dt;
	  //steer_value is negative
	  epsi1 += - v * steer_value / Lf * dt;	  
      	  Eigen::VectorXd state(6);	  
      	  state << x1,y1,psi1,v1,cte1,epsi1;
	  auto vars = mpc.Solve(state, coeffs);
      
      	  //print polynomial back to simulator
      	  vector<double> way_x_vals;
      	  vector<double> way_y_vals;  
      	  //x value distance
      	  const double poly_inc = Lf;
      	  //number of points to print
      	  const int num_points = 25;
      	  for(unsigned int i=1; i<num_points; i++){
      	    way_x_vals.push_back(poly_inc*i);
      	    way_y_vals.push_back(polyeval(coeffs, poly_inc*i));
      	  }
      
      	  vector<double> mpc_x_vals;
      	  vector<double> mpc_y_vals;
      	  for(unsigned int i=2; i<vars.size(); i++){
      	    if(i%2==0){
      	      mpc_x_vals.push_back(vars[i]);
      	    } else {
      	      mpc_y_vals.push_back(vars[i]);
      	    }
      	  }
      
      	  
	  // Calculate steering angle and throttle using MPC.
          json msgJson;
          msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf);
          msgJson["throttle"] = vars[1];

	  //Display the MPC predicted trajectory           
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          msgJson["next_x"] = way_x_vals;
          msgJson["next_y"] = way_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //        
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

	  if(iters > max_iters){
	    //Plot Graph for analysis of the first 100 iterations
	    plt::subplot(3, 1, 1);
	    plt::title("CTE");
	    plt::plot(cte_vals);
	    plt::subplot(3, 1, 2);
	    plt::title("Delta (Radians)");
	    plt::plot(delta_vals);
	    plt::subplot(3, 1, 3);
	    plt::title("Velocity");
	    plt::plot(v_vals);	    
	    plt::show();
	    iters = 0;
	    exit(1);
	  }
	  
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
