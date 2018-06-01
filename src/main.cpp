#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <iomanip>

#define throttlepid 0

using std::cout;
using std::endl;
using std::setw;

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
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char* argv[])
{
  uWS::Hub h;
  static int connect_count = 0;
  static double tol = 0.00001;
  static bool optimized = true;
  std::string save_to_file = "pid_params_output.txt";
  std::string reload_file = "";
  static double prev_cte = 0.0;
  PID steering_pid;
  static double thrtl_Kp = 0.0;
  static double thrtl_Ki = 0.0;
  static double thrtl_Kd = 0.0;
  static double steer_Kp = 0.061751;
  static double steer_Ki = 0.004;
  static double steer_Kd = -0.662543 ;
  static double max_speed = 35;
  static bool simreset = false;
  std::string pidStr = "";
  for(int args =1;args<argc;args=args+2){
	  pidStr = argv[args];
	  if(pidStr == "-tweedle")  {optimized = false; args--;}
	  else if (pidStr == "-tp") {thrtl_Kp = atof(argv[args+1]);}
	  else if (pidStr == "-ti") {thrtl_Ki = atof(argv[args+1]);}
	  else if (pidStr == "-td") {thrtl_Kd = atof(argv[args+1]);}
	  else if (pidStr == "-sp") {steer_Kp = atof(argv[args+1]);}
	  else if (pidStr == "-si") {steer_Ki = atof(argv[args+1]);}
	  else if (pidStr == "-sd") {steer_Kd = atof(argv[args+1]);}
	  else if (pidStr == "-mv") {max_speed = atof(argv[args+1]);}
	  else if (pidStr == "-savetofile") {save_to_file = argv[args+1];}
	  else if (pidStr == "-reload") {if (args+1 < argc) reload_file = argv[args+1]; else reload_file = "pid_params_input.txt";}
  }
#if throttlepid
	  PID throttle_pid;
	  throttle_pid.Init(thrtl_Kp,thrtl_Ki,thrtl_Kd);
#endif

  steering_pid.Init(steer_Kp,steer_Ki,steer_Kd);
  if (!optimized){
	  cout << "Tweedle optimization mode " << endl;
	  if(!steering_pid.setfile(save_to_file)) {cout << "error opening file to save pid params, execution stopped! " << endl; return(-1);};
	  if (reload_file != "") {
		  if (!steering_pid.loadParams(reload_file)) {cout << "Error reading param file " << reload_file << endl; return(-2);};
		  cout << "Loaded Tweedle PID params " << endl;
		  cout << "Kp " << steering_pid.Kp_ << endl;
		  cout << "Ki " << steering_pid.Ki_ << endl;
		  cout << "Kd " << steering_pid.Kd_ << endl;
		  cout << "dp[0] " << steering_pid.dp[0] << std::endl;
		  cout << "dp[1] " << steering_pid.dp[1] << std::endl;
		  cout << "dp[2] " << steering_pid.dp[2] << std::endl;
		  cout << "err " << steering_pid.prt_err() << std::endl;
		  cout << "Best_err " << steering_pid.prt_besterr() << std::endl;
		  cout << "State " << steering_pid.prt_state() << endl;
		  cout << "Next State " << steering_pid.prt_next_state() <<  endl;
		  cout << "Resume State " << steering_pid.prt_resume_state() << endl;
	  }
  } else {
#if throttlepid
	  std::cout << " thrtl_Kp " << thrtl_Kp << " thrtl_Ki " << thrtl_Ki << " thrtl_Kd " << thrtl_Kd << endl;
#endif
	  cout << "steer_Kp  " << steer_Kp << endl;
	  cout << "steer_Ki  " << steer_Ki << endl;
	  cout << "steer_Kd  " << steer_Kd << endl;
	  cout << "Max_Speed " << max_speed << endl;

  }
  h.onMessage([&steering_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string msg = "";
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle;
          if (simreset) { // wait till first after reset message is received
        	  if (angle == 0) simreset =false;
          }
          steering_pid.UpdateError(cte); //3*cte/(log(1+speed/2)));

          // simpler alternative to a PID for the throttle, with the benefit it can in fact rapidly reduce speed, in case of large cte.
    	  throttle = (0.7-fabs(cte -prev_cte))*fabs(1-(speed/max_speed));
		  json msgJson;
    	  if (!optimized && !simreset) { // if not optimized, run Tweedle state machine.
    		  // when the state machine resets the simulator, the queue should be cleared ...
    		  // no time to figure this out ... simply avoid using the state machine until the first after reset message is received ..
			  optimized = steering_pid.Tweedle(tol);
			  if (steering_pid.twdl_loops % 6 == 0 ) {cout <<"\b\b\b\b\b* " << setw(3) << int(steering_pid.twdl_loops/600.0 *100) << "%"; cout.flush();}
			  if (steering_pid.twdl_loops % 60 == 0 ) {cout <<" "; cout.flush();}
			  // Detect crash >> the tweedle state machine estimating the PID params needs to know when a crash
              // occurred, so it can restart the simulator.
			  // Either one of the following criteria triggers a crash condition
			  // (Speed < 0.1 && cte > 1)
			  // or
			  // fabs(cte > 4), which is essentially when one wheel goes over the curb
			  // Detecting a crash is necessary to restart the simulator automatically
			  // If the car remains on the track, the Tweedle state machine will runs the error estiamtion for about 600 steps , which is roughly one full
			  // track loop, before proceeding to the best_err comparison

			  // The simulator restart is called by the Tweedle State machine so it can reset its internal counter and
			  // state variables to continue the estimation process uninterrupted, after restarting the simulator
			  if ((speed < 0.1 && fabs(cte) > 1.0) || (fabs(cte) > 4.0)) {// either the car got stuck
				  steering_pid.simRestart();

				  msg = "42[\"reset\",{}]";
				  simreset = true;
			  } else {
				  steer_value = steering_pid.TotalError();
	#if throttlepid
				  throttle_pid.UpdateError(speed -target_speed);
				  double throttle_test = throttle_pid.TotalError();
				  cout << "Throttle PID value  " << throttle_test;
	#endif
				  if (steer_value > 1){
					  steer_value =1;
				  }
				  if (steer_value < -1){
					  steer_value = -1;
				  }
				  prev_cte = cte; // not used
				  if (simreset) {
					  steer_value = 0.0;
					  throttle = 0.0;
				  }
				  msgJson["steering_angle"] = steer_value;
				  msgJson["throttle"] = throttle;
				  msg = "42[\"steer\"," + msgJson.dump() + "]";
				  //std::cout << msg << std::endl;

			  }

		  } else { // no optimization needed
			  steer_value = steering_pid.TotalError();

#if throttlepid
			  throttle_pid.UpdateError(speed -target_speed);
			  double throttle_test = throttle_pid.TotalError();
			  cout << "Throttle PID value  " << throttle_test;
#endif
			  if (steer_value > 1){
				  steer_value =1;
			  }
			  if (steer_value < -1){
				  steer_value = -1;
			  }
			  prev_cte = cte;

			  msgJson["steering_angle"] = steer_value;
			  msgJson["throttle"] = throttle;
			  msg = "42[\"steer\"," + msgJson.dump() + "]";
			  //std::cout << msg << std::endl;
		  }

    	  //if (reseton) {std::cout << "MESSAGE OUT " << endl << msg << std::endl;}
		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		if (connect_count < 1) {
			std::cout << "Connected!!!" << std::endl;
			connect_count++;
		}
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
	std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
