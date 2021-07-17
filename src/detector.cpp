#include "detector.hpp"

detector::detector() {
	vid = 0;
	vc.open(vid);
	if(!vc.isOpened()) {
		std::cerr << "Failed to open video input with index " << vid << "\n";
		return;
	}
	vi ids;
	vvp2f corners;
	cv::Ptr<cv::aruco::Dictionary> dict =
		cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(16));
	char key;
	while(vc.grab()) {
		cv::Mat img, cpy;
		vc.retrieve(img);
		img.copyTo(cpy);

		ids.clear();
		corners.clear();

		cv::aruco::detectMarkers(img, dict, corners, ids);

		if(ids.size() > 0)
			cv::aruco::drawDetectedMarkers(cpy, corners, ids);

		cv::putText(cpy, "Press <Esc> to Exit", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
		cv::imshow("Detected Markers", cpy);

		if(27 ^ (char)cv::waitKey(wtime))
			continue;
		break;
	}
	vc.release();
}

inline void detector::set_input(int vid) {
	this->vid = vid;
}