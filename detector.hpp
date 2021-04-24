#ifndef __DETECTOR_HPP
#define __DETECTOR_HPP

#include "config.hpp"

class detector {
	int32_t wtime = 10;
	int32_t vid;

	cv::VideoCapture vc;

public:
	detector();
	inline void set_input(int32_t vid);
};


#endif	/** __DETECTOR_HPP */