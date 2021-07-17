#include "create_board.hpp"

create_board::create_board() {
	n_markers = {2, 4};
	len = 200;
	padding = 100;
	margin = 100;
	dict_id = 16;
	border_bits = 1;
	out_file = "board.jpg";
}

void create_board::generate_board(bool show) {
	img_size = {n_markers.second * (len + padding) - padding + 2 * margin,
		n_markers.first * (len + padding) - padding + 2 * margin};

	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dict_id));
	gboard = cv::aruco::GridBoard::create(n_markers.second, n_markers.first, len, padding, dictionary);

	gboard->draw(img_size, board, margin, border_bits);
	cv::imwrite(out_file, board);

	if(show) {
		cv::imshow("Board", board);
		cv::waitKey(0);
	}
}