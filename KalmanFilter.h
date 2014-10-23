#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_


class KalmanFilter {

public :

KalmanFilter();
void Configure(float zVariance, float zAccelVariance, float zAccelBiasVariance, float zInitial, float vInitial, float aBiasInitial);
void Update(float z, float a, float dt, float* pZ, float* pV);

private :

// State being tracked
	float z_;  // position
	float v_;  // velocity
	float aBias_;  // acceleration

// 3x3 State Covariance matrix
	float Pzz_;
	float Pzv_;
	float Pza_;
	float Pvz_;
	float Pvv_;
	float Pva_;
	float Paz_;
	float Pav_;
	float Paa_;

    float zAccelBiasVariance_; // assumed fixed.
	float zAccelVariance_;  // dynamic acceleration variance
	float zVariance_; //  z measurement noise variance fixed

};

#endif

