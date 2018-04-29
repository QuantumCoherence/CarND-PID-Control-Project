#include "PID.h"
#include <string>
#include <iostream>
using namespace std;

PID::PID() {
	p[0] = 0;
	p[1] = 0;
	p[2] = 0;
	dp[0] = 0.3;
	dp[1] = 0.3;
	dp[2] = 0.3;

	err = 0;
	best_err = 0;
	pid_i    = 0;
	prevCte  = 0;
	p_error_ = 0.0;
	i_error_ = 0.0;
	d_error_ = 0.0;
	twdl_state = ERROR_INIT;
	twdl_loops = 1;
	err        = 0;
	best_err   = 0;
	err_runs_  = 35;
	max_loops = err_runs_;
	reset_max_loops = false;

}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;

}

void PID:: setErrorRuns(int n){
	err_runs_ = n;
}
void PID::UpdateError(double cte) {
	p_error_  = cte;
	i_error_ += cte;
	d_error_  = prevCte - cte;
	prevCte   = cte;
}

double PID::TotalError() {

	return -Kp_*p_error_ -Kd_ * d_error_ - Ki_ * i_error_;
}

void PID::exitErrorUpdateState(){
	if (twdl_state == ERROR_UPDATE) {
		max_loops = twdl_loops+1;
		reset_max_loops = true;
	}
}

bool PID::Tweedle(double tol) {
	bool done = false;
	switch(twdl_state)
	{ case ERROR_INIT:{
	  	  best_err += p_error_*p_error_;
	  	  twdl_loops++;
	  	  if (twdl_loops >= max_loops){
	  		best_err /= max_loops;
	  		twdl_state = START;
	  		transition_state = NONE;
	  		twdl_loops = 1;
			if (reset_max_loops) {
				max_loops = err_runs_;
				reset_max_loops = false;
			}
	  	  }
		}
		break;
	  case START:{
			pid_i = 0;
			if (dp[0]+dp[1]+dp[2] > tol) {
				p[pid_i] += dp[pid_i];
				twdl_state = ERROR_UPDATE;
				transition_state = TOL0DP;
			} else {
				twdl_state = END;
			}
		}
		break;
	  case ERROR_UPDATE:{
		  	  if(twdl_loops == 0) {err = 0;}
		  	  err += p_error_*p_error_;
		  	  twdl_loops++;
		  	  if (twdl_loops >= max_loops){
		  		err /= max_loops;
		  		twdl_state = transition_state;
		  		transition_state = NONE;
		  		twdl_loops = 1;
				if (reset_max_loops) {
					max_loops = err_runs_;
					reset_max_loops = false;
				}
		  	  }
	  	  }
		  break;
	  case TOL0DP:{
		  	  if (err < best_err){
		  		  best_err = err;
		  		  dp[pid_i]*= 1.1;
		  		  pid_i = 1;
		  		  p[pid_i] += dp[pid_i];
				  twdl_state = ERROR_UPDATE;
				  transition_state = TOL1DP;
		  	  } else {
		  		  p[pid_i] -= 2* dp[pid_i];
				  twdl_state = ERROR_UPDATE;
				  transition_state = TOL0_2DP;
		  	  }
		   }
		   break;
	  case TOL1DP:{
		  	  if (err < best_err) {
		  		  best_err = err;
		  		  dp[pid_i] *= 1.1;
		  		  pid_i = 2;
		  		  p[pid_i] += dp[pid_i];
				  twdl_state = ERROR_UPDATE;
				  transition_state = TOL2DP;
		  	  } else {
		  		  p[pid_i]-= 2 * dp[pid_i];
				  twdl_state = ERROR_UPDATE;
				  transition_state = TOL1_2DP;
		  	  }
			}
		   break;
	  case TOL0_2DP:{
		  	  if (err < best_err){
		  		  best_err = err;
		  		  dp[pid_i] *= 1.1;
		  	  } else {
		  		  p[pid_i]+=dp[pid_i];
		  		  dp[pid_i] *= 0.9;
		  	  }
	  		  pid_i = 1;
	  		  p[pid_i] += dp[pid_i];
			  twdl_state = ERROR_UPDATE;
			  transition_state = TOL1DP;
		   }
		   break;
	  case TOL1_2DP:{
			  if (err < best_err){
				  best_err = err;
				  dp[pid_i] *= 1.1;
			  } else {
				  p[pid_i]+=dp[pid_i];
				  dp[pid_i] *= 0.9;
			  }
			  pid_i = 2;
			  p[pid_i] += dp[pid_i];
			  twdl_state = ERROR_UPDATE;
			  transition_state = TOL2DP;
		   }
		   break;
	  case TOL2DP:{
		  	  if (err < best_err){
		  		  dp[pid_i] *= 1.1;
		  		  pid_i = 0;
				  twdl_state = END;
		  	  } else {
		  		  p[pid_i] += dp[pid_i];
		  		  dp[pid_i] *= 0.9;
		  		  pid_i=0;
				  twdl_state = END;
		  	  }
		   }
		   break;
	   case END:{
		   pid_i = 0;
			if (dp[0]+dp[1]+dp[2] > tol) {
				done = false;
				twdl_state = START;
				std::cout << "\ttol " <<  dp[0]+dp[1]+dp[2];
			} else {
				done = true;
			}
		   }
		   break;
	   default:{
		   twdl_state = ERROR;
		   done = false;
		   }
		   break;
	} /*switch*/
	Kp_ = p[0];
	Ki_ = p[1];
	Kd_ = p[2];
	return done;
}

double PID::prt_err(){
	return err/twdl_loops;
}
double PID::prt_besterr(){
	return best_err;
}
string PID::prt_state(){

	switch(twdl_state){
		case ERROR_INIT:
			return "ERROR_INIT";
			break;
		case START:
			return "START";
			break;
		case TOL0DP:
			return "TOL0DP";
			break;
		case TOL0_2DP:
			return "TOL0_2DP";
			break;
		case TOL1DP:
			return "TOL1DP";
			break;
		case TOL1_2DP:
			return "TOL1_2DP";
			break;
		case TOL2DP:
			return "TOL2DP";
			break;
		case END:
			return "END";
			break;
		case ERROR:
			return "ERROR";
			break;
		case ERROR_UPDATE:
			return "ERROR_UPDATE";
			break;
		default:
			return "UNKNOWN";
			break;

		}
}
