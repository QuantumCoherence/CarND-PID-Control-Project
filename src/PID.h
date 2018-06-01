#ifndef PID_H
#define PID_H
#include <string>
#include <fstream>

// Tweedle State Machine state labels
const int ERROR_INIT = 0;
const int START      = 1;
const int TOL0DP     = 2;
const int TOL0_2DP   = 3;
const int TOL1DP     = 4;
const int TOL1_2DP   = 5;
const int TOL2DP     = 6;
const int TOL2_2DP   = 7;
const int END        = 8;
const int ERROR      = 9;
const int ERROR_UPDATE =10;
const int NONE       =  11;




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
  std::string prt_state();
  std::string prt_next_state();
  std::string prt_resume_state();
  double prt_err();
  double prt_besterr();
  int twdl_loops;
  void simRestart();
  void saveParams();
  bool loadParams(std::string filename);
  bool setfile(std::string filename);
  bool fileExists(const std::string& filename);
  // I am being lazy .. this should be private
  double dp[3];

private:

    std::ofstream pid_state_file;
    double p[3];
	double prevCte;
	double err;
	double best_err;
	int pid_i;
	int twdl_state;
	int next_state;
	int resume_state;
	int max_loops;
	bool reset_max_loops;

};

#endif /* PID_H */
