#include "PID.h"
#include <string>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <stdio.h>
using namespace std;
using std::setw;

PID::PID() {
	p[0] = 0;
	p[1] = 0;
	p[2] = 0;
	dp[0] = 0.1;
	dp[1] = 0.1;
	dp[2] = 0.1;

	err = 0;
	best_err = 1000;
	pid_i    = 0;
	prevCte  = 0;
	p_error_ = 0.0;
	i_error_ = 0.0;
	d_error_ = 0.0;
	twdl_state = START;
	twdl_loops = 1;
	err        = 0;
	err_runs_  = 600;
	max_loops = err_runs_;
	reset_max_loops = false;
	next_state = NONE;
	resume_state = NONE;
	Kp_ = 0;
	Ki_ = 0;
	Kd_ = 0;

}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Kd_ = Kd;
	Ki_ = Ki;
	p[0] = Kp;
	p[1] = Kd;
	p[2] = Ki;

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

  return -Kp_*p_error_ -Kd_* d_error_ - Ki_* i_error_;
}

// Reset the car back to starting position, and it can be used in tweedle
void PID::simRestart(){

	switch (twdl_state)
	{  	case ERROR_UPDATE: {
			err /= twdl_loops;
			twdl_state = next_state;
			next_state = NONE;
			resume_state = NONE;
			twdl_loops = 1;
		}
		break;
		default: {

		}
	}
	cout << "\r"; cout.clear();cout << "                                                                                                                     "; cout.flush();
	cout << "\rReset ";
	cout << "\tTol: " <<  setw(10) << dp[0]+dp[1]+dp[2] << "\tKp= " <<  setw(10) << Kp_ << "\tKi= " <<  setw(10) << Ki_ << "\tKd= " <<  setw(10) << Kd_;
	cout <<  "\tState "  <<  setw(10) << this->prt_state() << "\tBest Error " <<  setw(10) << best_err << "\tError " <<  setw(10) << err << "\tCte " << p_error_ << endl;
	this->saveParams();

}


bool PID::Tweedle(double tol) {
	// Tweedle State machine
	// Note: the state transition logic is sequence dependent ...
	//
	bool done = false;
	switch(twdl_state)
	{ case ERROR_INIT:{ //used for debugging only
	  	  best_err += p_error_*p_error_;
	  	  twdl_loops++;
	  	  //cout << "increment loops " << twdl_loops << endl;
	  	  if (twdl_loops >= max_loops){
	  		best_err /= max_loops;
	  		twdl_state = START;
	  		next_state = NONE;
	  		resume_state = NONE;
	  		twdl_loops = 1;
			std::cout << "Tol " <<  dp[0]+dp[1]+dp[2] << " best_error " << best_err << endl;
			std::cout << "Kp= " << Kp_ << "\tKi= " << Ki_ << "\tKd- " << Kd_ << endl << endl;
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
				next_state = TOL0DP;
				resume_state = NONE;
			} else {
				twdl_state = END;
			}
		}
		break;
	  case ERROR_UPDATE:{
		      if (twdl_loops == 1) {err = 0; i_error_ = 0;}
 		  	  err += p_error_*p_error_;
		  	  twdl_loops++;
		  	  if (twdl_loops >= max_loops){
		  		err /= max_loops;
		  		twdl_state = next_state;
		  		next_state = NONE;
		  		resume_state = NONE;
		  		twdl_loops = 1;
		  		cout << "\r"; cout.clear();cout << "                                                                                                                     "; cout.flush();
		  		cout << "\rLoop  ";
		  		cout << "\tTol: " <<  setw(10) << dp[0]+dp[1]+dp[2] << "\tKp= " <<  setw(10) << Kp_ << "\tKi= " <<  setw(10) << Ki_ << "\tKd= " <<  setw(10) << Kd_;
		  		cout <<  "\tState "  <<  setw(10) << this->prt_state() << "\tBest Error " <<  setw(10) << best_err << "\tError " <<  setw(10) << err << "\tCte " << p_error_ << endl;
		  		this->saveParams();
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
				  next_state = TOL1DP;

		  	  } else {
		  		  p[pid_i] -= 2* dp[pid_i];
				  twdl_state = ERROR_UPDATE;
				  next_state = TOL0_2DP;
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
				  next_state = TOL2DP;
		  	  } else {
		  		  p[pid_i]-= 2 * dp[pid_i];
				  twdl_state = ERROR_UPDATE;
				  next_state = TOL1_2DP;
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
			  next_state = TOL1DP;
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
			  next_state = TOL2DP;
		   }
		   break;
	  case TOL2DP:{
		  	  if (err < best_err){
		  		  best_err = err;
		  		  dp[pid_i] *= 1.1;
		  		  pid_i = 0;
				  if (dp[0]+dp[1]+dp[2] > tol) {
					 done = false;
					 p[pid_i] += dp[pid_i];
 					 twdl_state = ERROR_UPDATE;
					 next_state = TOL0DP;
				  } else {
					 done = true;
   				     twdl_state = END;
				  }
		  	  } else {
		  		  p[pid_i]-= 2 * dp[pid_i];
				  twdl_state = ERROR_UPDATE;
				  next_state = TOL2_2DP;
		  	  }
		   }
		   break;
	  case TOL2_2DP:{
			  if (err < best_err){
				  best_err = err;
				  dp[pid_i] *= 1.1;
			  } else {
				  p[pid_i]+=dp[pid_i];
				  dp[pid_i] *= 0.9;
			  }
			  pid_i = 0;
			  if (dp[0]+dp[1]+dp[2] > tol) {
				 done = false;
				 p[pid_i] += dp[pid_i];
				 twdl_state = ERROR_UPDATE;
				 next_state = TOL0DP;
			  } else {
				 done = true;
				 twdl_state = END;
			  }
	  	  }
	      break;
	   case END:{
		    pid_i = 0;
			done = true;
			// save parameters
			cout << "\r"; cout.clear();cout << "                                                                                                                     "; cout.flush();
			cout << "\rFinal ";
			cout << "\tTol: " <<  setw(10) << dp[0]+dp[1]+dp[2] << "\tKp= " <<  setw(10) << Kp_ << "\tKi= " <<  setw(10) << Ki_ << "\tKd= " <<  setw(10) << Kd_;
			cout <<  "\tState "  <<  setw(10) << this->prt_state() << "\tBest Error " <<  setw(10) << best_err << "\tError " <<  setw(10) << err << "\tCte " << p_error_ << endl;
			this->saveParams();

		   }
		   break;
	   default:{
		   twdl_state = ERROR;
		   done = false;
		   }
		   break;
	} /*switch*/
	Kp_ = p[0];
	Kd_ = p[1];
	Ki_ = p[2];
	return done;
}

bool PID::setfile(std::string filename){

   pid_state_file.open(filename);
   if (!pid_state_file.is_open())
		 {
			 std::cout << "Something went wrong with opening the pid parameters file!";
			 return(false);
		 } else return(true);
 }

bool PID::loadParams(std::string filename){

	 if (!fileExists(filename)) {return(false);}
	 std::ifstream infile(filename.c_str());
 	 if (!infile){
		 std::cout << "Something went wrong with opening the reload pid parameters file!";
		 return(false);
	 } else {
		 std::string a;
		 double b;
		 int c;
		 infile >> a >> b; p[0] = b;
		 infile >> a >> b; p[2] = b;
		 infile >> a >> b; p[1] = b;
		 infile >> a >> b; dp[0] = b;
		 infile >> a >> b; dp[1] = b;
		 infile >> a >> b; dp[2] = b;
		 infile >> a >> b; err = b;
		 infile >> a >> b; best_err = b;
		 infile >> a >> a >> c; twdl_state = c;
		 infile >> a >> a >> a >> c; next_state = c;
		 infile >> a >> a >> a >> c; resume_state = c;
		 Kp_= p[0];
		 Kd_= p[1];
		 Ki_= p[2];
		 infile.close();
		 return(true);
	 }

}

#include <sys/stat.h>
// Function: fileExists
/**
    Check if a file exists
@param[in] filename - the name of the file to check

@return    true if the file exists, else false

*/
bool PID::fileExists(const std::string& filename)
{
    struct stat buf;
    if (stat(filename.c_str(), &buf) != -1)
    {
        return true;
    }
    return false;
}

void PID::saveParams(){
	pid_state_file << "Kp " << Kp_ << std::endl;
	pid_state_file << "Ki " << Ki_ << std::endl;
	pid_state_file << "Kd " << Kd_ << std::endl;
	pid_state_file << "dp[0] " << dp[0] << std::endl;
	pid_state_file << "dp[1] " << dp[1] << std::endl;
	pid_state_file << "dp[2] " << dp[2] << std::endl;
	pid_state_file << "err " << this->prt_err() << std::endl;
	pid_state_file << "Best_err " << this->prt_besterr() << std::endl;
	pid_state_file << "State " << this->prt_state() << " " << twdl_state << std::endl;
	pid_state_file << "Next State " << this->prt_next_state() <<  " " << next_state << std::endl;
	pid_state_file << "Resume State " << this->prt_resume_state() <<  " " << resume_state << std::endl;
	pid_state_file.seekp(0, std::ios::beg);
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
		case TOL2_2DP:
			return "TOL2_2DP";
		case END:
			return "END";
			break;
		case ERROR:
			return "ERROR";
			break;
		case ERROR_UPDATE:
			return "ERROR_UPDATE";
			break;
		case NONE:
			return "NONE";
			break;
		default:
			return "UNKNOWN";
			break;

		}
}
string PID::prt_next_state(){

	switch(next_state){
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
		case TOL2_2DP:
			return "TOL2_2DP";
		case END:
			return "END";
			break;
		case ERROR:
			return "ERROR";
			break;
		case ERROR_UPDATE:
			return "ERROR_UPDATE";
			break;
		case NONE:
			return "NONE";
			break;
		default:
			return "UNKNOWN";
			break;

		}
}
string PID::prt_resume_state(){

	switch(resume_state){
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
		case TOL2_2DP:
			return "TOL2_2DP";
		case END:
			return "END";
			break;
		case ERROR:
			return "ERROR";
			break;
		case ERROR_UPDATE:
			return "ERROR_UPDATE";
			break;
		case NONE:
			return "NONE";
			break;
		default:
			return "UNKNOWN";
			break;

		}
}
