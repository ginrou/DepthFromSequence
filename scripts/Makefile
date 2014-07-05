MAIN=mock_test.cpp
DST_POINT= points.txt
CFLAGS= -O0

## build rules
bin/main.out: bin/main.o bin/mock_data_factroy.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

bin/main.o: $(MAIN)
	g++ $(MAIN) -c -o $@ $(CFLAGS) `pkg-config --cflags opencv` 

bin/mock_data_factroy.o: mock_data_factory.cpp
	g++ $^ -c -o $@ $(CFLAGS) `pkg-config --cflags opencv` 

## command
run: 
	./bin/main.out $(INPUT_IMAGES) $(DST_POINT)
and_run: $(MAIN) bin/main.o
	make
	make run

clean:
	rm bin/main.out bin/main.o bin/mock_data_factory.cpp
