processing:
	bash -c "g++ -g `pkg-config --cflags opencv` -o image_processing image_processing.cpp getTime.cpp `pkg-config --libs opencv` -lwiringPi"

benchmark:
	bash -c "g++ -g `pkg-config --cflags opencv` -o image_processing_benchmark image_processing_benchmark.cpp `pkg-config --libs opencv`"
	bash -c "./image_processing_benchmark"

reciever:
	bash -c "g++ -o image_reciever image_reciever.cpp"

run:
	bash -c "./image_processing -v | ffplay -loglevel 0 -" 
