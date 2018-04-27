#ifndef PID_H
#define PID_H
#include <vector>

// Tweendle State Machine state labels
const int START    = 0;
const int TOL0DP   = 1;
const int TOL0_2DP = 2;
const int TOL1DP   = 3;
const int TOL1_2DP = 4;
const int TOL2DP   = 5;
const int END      = 6;
const int ERROR    = 7;
const int ERROR_UPDATE = 8;
const int NONE     =  9;


class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;
  int err_runs_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * estimate PID params.
  */
  bool Tweedle(double tol);
  /*
  * sets the number of runs, for estimating the current best error, default to 100
  */
  void setErrorRuns(int n);

private:
    double p[3];
    double dp[3];
	double prevCte;
	double err;
	double best_err;
	int pid_i;
	int twdl_state;
	int transition_state;
	int twdl_loops;

std::string prt_lablel(int state);

};

#endif /* PID_H */
