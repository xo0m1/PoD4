# Command line build
#g++ -Wall -o "%e" "%f" $(pkg-config opencv --cflags --libs) -lraspicam -lraspicam_cv -L/opt/vc/lib -lmmal -lmmal_core -lmmal_util

CC = g++
CFLAGS = -c `pkg-config --cflags opencv` -Wall
OCVLIBS = `pkg-config --libs opencv`
LDFLAGS = -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util
LDPATH = -L/opt/vc/lib -L/usr/local/lib
#SOURCES = src/main_video_v2_2.cpp
#SOURCES = src/main.cpp src/blinkDetectModule_demo.cpp
SOURCES = src/main.cpp src/blinkDetectModule.cpp
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = blinkDetect


#% : %.cpp
#	g++ $(CFLAGS) $(OCVLIBS) -o $@ $<

#blink : $(SOURCES_BLINK) $(EXECUTABLE_BLINK)
#	$(CC) $(CFLAGS) $(OCVLIBS) -o $(EXECUTABLE) $<
	
	
all : $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDPATH) $(OCVLIBS) $(LDFLAGS) $(OBJECTS) -o $@
	@echo "Done - Blink"
	
	
.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm src/*.o $(EXECUTABLE)
