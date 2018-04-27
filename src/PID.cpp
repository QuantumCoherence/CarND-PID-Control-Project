#include "PID.h"

using namespace std;


PID::PID() {
	p[0] = 0;
	p[1] = 0;
	p[2] = 0;
	dp[0] = 0;
	dp[1] = 0;
	dp[2] = 0;

	err = 0;
	best_err = 0;
	pid_i    = 0;
	prevCte  = 0;
	p_error_ = 0.0;
	i_error_ = 0.0;
	d_error_ = 0.0;
	twdl_state = START;
	twdl_loops = 0;
	err        = 0;
	best_err   = 0;
	err_runs_  = 100;

}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
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

bool PID::Tweedle(double tol) {

	switch(twdl_state)
	{ case START:{
			pid_i = 0;
			if (dp[0]+dp[1]+dp[2] > tol) {
				p[pid_i] += dp[pid_i]
				twdl_state = ERROR_UPDATE;
				transition_state = TOL0DP;
			} else {
				twdl_state = END;
				pid_i = 0;
			}
		}
		break;
	  case ERROR_UPDATE:{
		  	  if(twdl_loops == 0) {err = 0;}
		  	  err += p_error_*p_error_;
		  	  twdl_loops++;
		  	  if (twdl_loops == err_runs_){
		  		err /= err_runs_;
		  		twdl_state = transition_state;
		  		transition_state = NONE;
		  		twdl_loops = 0;
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
		  		  pid_i = 1;
		  		  p[pid_i] += dp[pid_i];
				  twdl_state = ERROR_UPDATE;
				  transition_state = TOL1DP;
		  	  } else {
		  		  p[pid_i]+=dp[pid_i];
		  		  dp]pid_i] *= 0.9;
		  		  pid_i = 1;
		  		  p[pid_i] += dp[pid_i];
				  twdl_state = ERROR_UPDATE;
				  transition_state = TOL1DP;
		  	  }
		   }
		   break;
	  case TOL1_2DP:{
		   }
		   break;
	  case TOL2DP:{
		   }
		   break;
	   case END:{
		   }
		   break;
	   default:{
		   twdl_state = ERROR;
		   return false;
		   }
		   break;
	} /*switch*/

	if (dp[0]+dp[1]+dp[2] > tol) {
		return true;
	} else {
		return false;
	}
}

string PID::prt_lablel(int state){

	switch(state){
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
		default:
			return "UNKNOWN";
			break;

		}
}


