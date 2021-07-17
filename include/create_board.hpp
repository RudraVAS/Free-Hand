#ifndef __CREATE_BOARD
#define __CREATE_BOARD

#include "config.hpp"

class create_board {
	std::pair<int32_t, int32_t> n_markers;
	int32_t len;
	int32_t padding;
	int32_t margin;
	int32_t dict_id;
	int32_t border_bits;
	std::string out_file;
	cv::Size2i img_size;
	cv::Mat board;
	cv::Ptr<cv::aruco::Dictionary> dictionary;
	cv::Ptr<cv::aruco::GridBoard> gboard;

public:
	create_board();
	void generate_board(bool show = false);
};


#endif	/** __CREATE_BOARD */