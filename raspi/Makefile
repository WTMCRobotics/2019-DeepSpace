all:
	bash -c "g++ -g `pkg-config --cflags opencv` -o image_processing image_processing.cpp `pkg-config --libs opencv` -lwiringPi"

run:
	bash -c "./image_processing -v | ffplay -loglevel 0 -" 