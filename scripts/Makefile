MAIN=main.cpp

DST_POINT= points.txt
CAM_TRANS_FILE=cam_trans.txt
CAM_ROT_FILE=cam_rot.txt

CFLAGS= -O3
INPUT_IMAGES=img/*.jpg

## build rules
## executable files
bin/main.out: bin/main.o bin/mock_data_factroy.o bin/bundle_adjustment.o bin/feature_tracking.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

bin/mock_test.out: bin/mock_test.o bin/mock_data_factroy.o bin/bundle_adjustment.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

bin/mock_test_2.out: bin/mock_test_2.o bin/mock_data_factroy.o bin/bundle_adjustment.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

## plane sweepのテスト
bin/mock_test_3.out: bin/mock_test_3.o bin/mock_data_factroy.o bin/bundle_adjustment.o bin/plane_sweep.o bin/mock_image_factory.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

bin/depth_estimation.out: bin/plane_sweep.o bin/depth_estimation.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

## object files
bin/main.o: $(MAIN)
	g++ $(MAIN) -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/mock_test.o: mock_test.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/mock_test_2.o: mock_test_2.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/mock_test_3.o: mock_test_3.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/depth_estimation.o: depth_estimation.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/mock_data_factroy.o: mock_data_factory.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/mock_image_factory.o: mock_image_factory.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/bundle_adjustment.o: bundle_adjustment.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/feature_tracking.o: feature_tracking.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/plane_sweep.o: plane_sweep.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

## command
run:
	./bin/main.out $(INPUT_IMAGES) $(DST_POINT)

and_run: $(MAIN) bin/main.o
	make
	make run

mock_test: bin/mock_test.out
	./bin/mock_test.out $(DST_POINT)

mock_test_2: bin/mock_test_2.out
	./bin/mock_test_2.out $(DST_POINT)

mock_test_3: bin/mock_test_3.out
	./bin/mock_test_3.out $(DST_POINT)

run_depth_estimation: bin/depth_estimation.out
	./bin/depth_estimation.out $(INPUT_IMAGES) $(CAM_TRANS_FILE) $(CAM_ROT_FILE)

clean:
	rm bin/*.o bin/*.out
