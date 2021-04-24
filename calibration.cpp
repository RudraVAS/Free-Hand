#include "calibration.hpp"

bool camera_calibration::__read_dconf() {
	cv::FileStorage fs(detect_config, cv::FileStorage::READ);

	if(!fs.isOpened()) return false;

	fs["adaptiveThreshWinSizeMin"] >> detector_params->adaptiveThreshWinSizeMin;
	fs["adaptiveThreshWinSizeMax"] >> detector_params->adaptiveThreshWinSizeMax;
	fs["adaptiveThreshWinSizeStep"] >> detector_params->adaptiveThreshWinSizeStep;
	fs["adaptiveThreshConstant"] >> detector_params->adaptiveThreshConstant;
	fs["minMarkerPerimeterRate"] >> detector_params->minMarkerPerimeterRate;
	fs["maxMarkerPerimeterRate"] >> detector_params->maxMarkerPerimeterRate;
	fs["markerBorderBits"] >> detector_params->markerBorderBits;
	fs["perspectiveRemovePixelPerCell"] >> detector_params->perspectiveRemovePixelPerCell;
	fs["perspectiveRemoveIgnoredMarginPerCell"] >> detector_params->perspectiveRemoveIgnoredMarginPerCell;
	fs["maxErroneousBitsInBorderRate"] >> detector_params->maxErroneousBitsInBorderRate;
	fs["polygonalApproxAccuracyRate"] >> detector_params->polygonalApproxAccuracyRate;
	fs["minCornerDistanceRate"] >> detector_params->minCornerDistanceRate;
	fs["minDistanceToBorder"] >> detector_params->minDistanceToBorder;
	fs["minMarkerDistanceRate"] >> detector_params->minMarkerDistanceRate;
	fs["cornerRefinementMethod"] >> detector_params->cornerRefinementMethod;
	fs["cornerRefinementWinSize"] >> detector_params->cornerRefinementWinSize;
	fs["cornerRefinementMaxIterations"] >> detector_params->cornerRefinementMaxIterations;
	fs["cornerRefinementMinAccuracy"] >> detector_params->cornerRefinementMinAccuracy;
	fs["minOtsuStdDev"] >> detector_params->minOtsuStdDev;
	fs["errorCorrectionRate"] >> detector_params->errorCorrectionRate;
	return true;
}

bool camera_calibration::write_calib_config(const std::string & filename, cv::Size imageSize,
			float aspectRatio, int flags,
			const cv::Mat & cameraMatrix,
			const cv::Mat & distCoeffs, double totalAvgErr) {
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
		return false;

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;

	if (flags & cv::CALIB_FIX_ASPECT_RATIO)
		fs << "aspectRatio" << aspectRatio;

	if (flags != 0) {
		sprintf(buf, "flags: %s%s%s%s",
			flags & cv::CALIB_USE_INTRINSIC_GUESS ?
			"+use_intrinsic_guess" : "",
			flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio"
			: "",
			flags & cv::CALIB_FIX_PRINCIPAL_POINT ?
			"+fix_principal_point" : "",
			flags & cv::CALIB_ZERO_TANGENT_DIST ?
			"+zero_tangent_dist" : "");
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;

	return true;
}

camera_calibration::camera_calibration(float l_marker, float p_marker) {
	n_markers = {2, 4};
	dict_id = 16;
	this->l_marker = l_marker;
	this->p_marker = p_marker;

	detector_params = cv::aruco::DetectorParameters::create();

	cam_id = 0;

	detect_config = "configs/detector_params.yml";
	calib_config = "configs/calib_params.yml";

	if(!__read_dconf()) {
		std::cerr << "Invalid Detector Parameter File : " << detect_config << "\n";
		exit(0);
	}
}

camera_calibration::camera_calibration() {
	f_aspct_rtio = false;
	
	n_markers = {2, 4};
	dict_id = 16;
	l_marker = 0.0529167f;
	p_marker = 0.0264583f;

	detector_params = cv::aruco::DetectorParameters::create();

	cam_id = 0;

	detect_config = "configs/detector_params.yml";
	calib_config =  "configs/calib_params.yml";

	if(!__read_dconf()) {
		std::cerr << "Invalid Detector Parameter File : " << detect_config << "\n";
		exit(0);
	}
}

void camera_calibration::fix_aspect_ratio(float aspect_ratio) {
	f_aspct_rtio = true;
	this->aspect_ratio = aspect_ratio;

	calib_flags |= cv::CALIB_FIX_ASPECT_RATIO;
}

inline void camera_calibration::set_zt_flag() {
	calib_flags |= cv::CALIB_ZERO_TANGENT_DIST;
}

inline void camera_calibration::set_PC_flag() {
	calib_flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
}

void camera_calibration::init_calib() {
	cv::VideoCapture vc;
	if(!vc.open(cam_id)) {
		std::cerr << "Can not open Video Input with index " << cam_id << "\n";
		exit(0);
	}

	cv::Ptr<cv::aruco::Dictionary> dict;
	cv::Ptr<cv::aruco::GridBoard> gboard;
	cv::Ptr<cv::aruco::Board> board;

	dict = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dict_id));
	gboard = cv::aruco::GridBoard::create(n_markers.second, n_markers.first, l_marker, p_marker, dict);
	board = gboard.staticCast<cv::aruco::Board>();

	vvvp2f corners;
	vvi ids;

	cv::Size img_size;
	int wtime = 10;
	while(vc.grab()) {
		cv::Mat img, cpy;
		vc.retrieve(img);

		vi _ids;
		vvp2f _corners, _rejected;

		cv::aruco::detectMarkers(img, dict, _corners, _ids, detector_params, _rejected);

		img.copyTo(cpy);
		if(_ids.size() > 0)
			cv::aruco::drawDetectedMarkers(cpy, _corners, _ids);
		
		cv::putText(cpy, "Camera Calibration", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
		cv::imshow("Calibration", cpy);
		
		switch(cv::waitKey(wtime)) {
		case 27: goto out;
		case 'c':
			if(_ids.size() <= 0) {
				std::cerr << "No markers in the frame. Please try again.\n";
				break;
			}

			std::cout << "Captured" << "\n";
			corners.push_back(_corners);
			ids.push_back(_ids);
			img_size = img.size();
		}
	}
	out:
	if(ids.size() < 1) {
		std::cerr << "No markers were captured.\nPlead try again.\n";
		return;
	}
	cv::Mat cam_mat, dist_coeff;
	vm rvecs, tvecs;

	double rep_err;

	if(calib_flags & cv::CALIB_FIX_ASPECT_RATIO) {
		cam_mat = cv::Mat::eye(3, 3, CV_64F);
		cam_mat.at<double>(0, 0) = aspect_ratio;
	}

	vvp2f corners_cnct;
	vi ids_cnct;
	vi marker_cntr_pframe;

	marker_cntr_pframe.reserve(corners.size());

	for(uint32_t i = 0; i < corners.size(); i++) {
		marker_cntr_pframe.push_back((int)corners[i].size());

		for(uint32_t j = 0; j < corners[i].size(); j++) {
			corners_cnct.push_back(corners[i][j]);
			ids_cnct.push_back(ids[i][j]);
		}
	}

	rep_err = cv::aruco::calibrateCameraAruco(corners_cnct, ids_cnct,
		marker_cntr_pframe, board, img_size, cam_mat, dist_coeff,
		rvecs, tvecs, calib_flags);

	if(!write_calib_config(calib_config, img_size, aspect_ratio, calib_flags,
		cam_mat, dist_coeff, rep_err)) {
		std::cerr << "Error writing calibration config file!\n";
		return;
	}

	std::cout << "Rep Error : " << rep_err << "\n"
		<< "Calibration config successfully saved to " << calib_config << "\n";
}