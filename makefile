all:
	g++ ./*.cpp -g `pkg-config opencv --cflags --libs`