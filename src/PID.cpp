#include "PID.h"
#include <iostream>
#include <vector>
#include "math.h"


//Initialization for twiddle
std::vector<double> p;
std::vector<double> best_p;
bool twiddle = false; //whether using twiddle flag
bool first = true; // first run flag
bool second = true; //second run flag 
double total_error = 0; //initial total error 
double best_error = 100000; //best error
double error = 0; // average cte
double tol = 0.001; //tolerance
int counter = 0; //counter
int n_max = 200; //time span amount
int n = 0; //parameter tuned
double dp[3] = {0.01,0.001,0.1};


PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  best_p = p;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error; 
  p_error = cte;
  i_error += cte; 
  
  if (twiddle){
  counter+=1;
  total_error += fabs(cte);
 
  if (counter > n_max) {
    error = total_error / n_max;        //average error
    /*set best error and best value at first sample*/
    if (first) {
        p[n]+=dp[n];
        first = false;
    } else {
        error = total_error / n_max;
        /* if new error is better than best error*/
        if (error<=best_error && second){
            best_error = error;         //set best error to error
            best_p[n]=p[n];
            dp[n]*=1.1;                 //modify dp to a slightly larger value 1.1
            p[n]+=dp[n];
        }
        else {                          //error is bigger than best_error
            if (second) {
                p[n]-=2*dp[n];          //subtract dp from p twice, because we added it before
                second = false;
            } else {                    //third try, see if continued improvement or not
                if (error<best_error){
                    best_error = error;
                    best_p[n]=p[n];
                    dp[n]*=1.1;
                } else {
                    p[n] += dp[n];      //goback to original value
                    dp[n]*=0.8;         //decrease step size
                }
                n+=1;                   //after 3 times through, go to the next parameter to optimize
                if (n>2){n=0;}          //if went through all 3 constants, set back to the first
                /*reset flgs*/
                first = true; 
                second = true;
            }
        }
    }
    /*reset counter and error*/
    counter=0;
    error = 0;
    
    //check if twiddle has sufficiently run its course
    double sum_dp = dp[0]+dp[1]+dp[2];
    if (sum_dp<tol){
      twiddle = false; //turn off twiddle
      std::cout << "Best values: " << best_p[0]<< ", " << best_p[1] << ", " << best_p[2] << std::endl;
    } 
  }//end cycle
 }
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return -kp*p_error - ki*i_error- kd* d_error;
}