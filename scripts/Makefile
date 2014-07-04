MAIN=bundleAdjustment_mock_test.cpp
INPUT_IMAGES= img/*
DST_IMAGE= disp.png
DST_POINT= points.txt
## AKAZE_LIBS= /usr/local/akaze/lib/libAKAZE.a
## AKAZE_FLAG= -I/usr/local/akaze/include
AKAZE_LIBS=
AKAZE_FLAG=
CFLAGS= -O0 $(AKAZE_FLAG)

## build rules
bin/main.out: bin/main.o bin/bundle_adjustment.o bin/bundle_adjustment_functions.o bin/feature_tracking.o bin/mock_data_factroy.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

bin/main.o: $(MAIN)
	g++ $(MAIN) -c -o $@ $(CFLAGS) `pkg-config --cflags opencv` 

bin/calib.o: calibration.cpp
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS)

bin/imagelist_creator.o: imagelist_creator.cpp
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS)

bin/bundle_adjustment.o: bundle_adjustment.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv` 

bin/bundle_adjustment_functions.o: bundle_adjustment_functions.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv` 

bin/feature_tracking.o: feature_tracking.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv` 

bin/mock_data_factroy.o: mock_data_factory.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv` 


## command
run: 
	./bin/main.out $(INPUT_IMAGES) $(DST_POINT)
and_run: $(MAIN) bin/main.o
	make
	make run

clean:
	rm bin/main.out bin/main.o bin/bundle_adjustment.o
