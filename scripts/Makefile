MAIN=depthFromSequence.cpp
INPUT_IMAGES= img/*.jpg
DST_IMAGE= disp.png
## AKAZE_LIBS= /usr/local/akaze/lib/libAKAZE.a
## AKAZE_FLAG= -I/usr/local/akaze/include
AKAZE_LIBS=
AKAZE_FLAG=
CFLAGS= -O0 $(AKAZE_FLAG)

## build rules
bin/main.out: bin/main.o
	g++ -o $@ $^ `pkg-config --cflags opencv` `pkg-config --libs opencv` $(CFLAGS) $(AKAZE_LIBS)

bin/main.o: $(MAIN)
	g++ $(MAIN) -c -o $@ $(CFLAGS) `pkg-config --cflags opencv` 


## command
run: 
	./bin/main.out $(INPUT_IMAGES) $(DST_IMAGE)
and_run: $(MAIN) bin/main.o
	make
	make run
