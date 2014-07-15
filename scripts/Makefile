MAIN=main.cpp
DST_POINT= points.txt
CFLAGS= -O0
INPUT_IMAGES=img/*.jpg

## build rules
## executable files
bin/main.out: bin/main.o bin/mock_data_factroy.o bin/bundle_adjustment.o bin/feature_tracking.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

bin/mock_test.out: bin/mock_test.o bin/mock_data_factroy.o bin/bundle_adjustment.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

bin/mock_test_2.out: bin/mock_test_2.o bin/mock_data_factroy.o bin/bundle_adjustment.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

## object files
bin/main.o: $(MAIN)
	g++ $(MAIN) -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/mock_test.o: mock_test.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/mock_test_2.o: mock_test_2.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/mock_data_factroy.o: mock_data_factory.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/bundle_adjustment.o: bundle_adjustment.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv`

bin/feature_tracking.o: feature_tracking.cpp
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

clean:
	rm bin/main.out bin/main.o bin/mock_data_factory.cpp bundle_adjustment.out
