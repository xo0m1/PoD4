# Command line build
#g++ -Wall -o "%e" "%f" $(pkg-config opencv --cflags --libs) -lraspicam -lraspicam_cv -L/opt/vc/lib -lmmal -lmmal_core -lmmal_util

CC = g++
#CFLAGS = -c `pkg-config --cflags opencv` -Wall
CFLAGS = -c -Wall
#OCVLIBS = `pkg-config --libs opencv`
#LDFLAGS = -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lrt -lpigpio -lpthread
LDFLAGS = -lrt -lpigpio -lpthread
LDPATH = -L/opt/vc/lib -L/usr/local/lib
SOURCES = src/main.c src/ads1015.c src/pulseSensor.c src/proximitySensor.c src/pressureSensor.c
#SOURCES = src/main_video_v2_2.cpp src/blink_detection_2.cpp src/ads1015.c src/pulseSensor.c
OBJECTS = $(SOURCES:.c=.o)
EXECUTABLE = drowsyDetect

#% : %.cpp
#	g++ $(CFLAGS) $(OCVLIBS) -o $@ $<

	
all : $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
#	$(CC) $(LDPATH) $(OCVLIBS) $(LDFLAGS) $(OBJECTS) -o $@
	$(CC) $(LDPATH) $(LDFLAGS) $(OBJECTS) -o $@
	@echo "Done"
	
	
.c.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm src/*.o $(EXECUTABLE)
