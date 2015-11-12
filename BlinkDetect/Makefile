# Command line build
#g++ -Wall -o "%e" "%f" $(pkg-config opencv --cflags --libs) -lraspicam -lraspicam_cv -L/opt/vc/lib -lmmal -lmmal_core -lmmal_util

CC = g++
CFLAGS = -c `pkg-config --cflags opencv` -Wall
OCVLIBS = `pkg-config --libs opencv`
LDFLAGS = -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lrt -lpigpio -lpthread
LDPATH = -L/opt/vc/lib -L/usr/local/lib
SOURCES = src/main.cpp src/ads1015.c src/pulseSensor.c src/blinkDetectModule.cpp src/proximitySensor.c src/pressureSensor.c
#SOURCES = src/main_video_v2_2.cpp src/blink_detection_2.cpp src/ads1015.c src/pulseSensor.c
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = drowsyDetect

#% : %.cpp
#	g++ $(CFLAGS) $(OCVLIBS) -o $@ $<

#blink : main.cpp
#	$(CC) $(CFLAGS) $(OCVLIBS) -o $(EXECUTABLE) $<
	
all : $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDPATH) $(OCVLIBS) $(LDFLAGS) $(OBJECTS) -o $@
	@echo "Done"
	
.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm src/*.o $(EXECUTABLE)
