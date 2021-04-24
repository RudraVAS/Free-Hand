#include <iostream>
#include <string>
#include <iomanip>

#include <sstream>
#include <vector>
#include <fstream>

using namespace std;
#pragma once

//Kalman Filter Function
class Kalman_Filter{
public:

double Kalman(double U)
{
    //constants (static)
    static const double R=40;  // noise covariance
    static const double H=1.0; // Measurement map scalar
    static double Q =10;  // INITIAL ESTIMATED COVARIANCE
    static double P =0;   // INITIAL ERROR COVARIANCE
    static double U_hat =0;// INITIAL ESTIMATED STATE
    static double K =0;  // INITIAL KALMAN GAIN
    
    // Update Kalman Gain
    K= P*H/(H*P*H+R);  
    // UPDATE ESTIMATED
    U_hat = U_hat + K*(U-H*U_hat);;
    
    //Update Error COVARIANCE
    P = (1-K*H)*P+Q;

    return U_hat;
}
};
//testing part
int main()
{
    double noise_val[5]={3.4444,23.5343,23.4344,23.543,95.4944};
    Kalman_Filter k;
    for(int i=0;i<5;i++)
    {
        cout<<"Noised Value"<<noise_val[i]<<"\n";
        double estimated_U=k.Kalman(noise_val[i]);
        cout<<"Estimated Value"<<estimated_U<<"\n";
        cout<<"-----------------------------------"<<"\n";
    }
    
    return 0;
}

