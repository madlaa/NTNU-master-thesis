# program executable name
TARGET = start
 
# compiler flags
CFLAGS = -g -Wall
 
# linker flags
LDFLAGS = -g -lrt -pthread -L/usr/local/lib -lgsl -lgslcblas -lm
 
# list of sources
SOURCES = rrulas.cpp \
	../kinematics/ur_kin.cpp \
	../ur_modern_driver-master/src/ur_driver.cpp \
	../ur_modern_driver-master/src/robot_state.cpp \
	../ur_modern_driver-master/src/do_output.cpp \
	../ur_modern_driver-master/src/robot_state_RT.cpp \
	../ur_modern_driver-master/src/ur_communication.cpp \
	../ur_modern_driver-master/src/ur_realtime_communication.cpp \
	../force/force.cpp
# default rule, to compile everything
all: $(TARGET)
 
# define object files
OBJECTS = $(SOURCES:.cpp=.o) \
 
# link programs
$(TARGET): $(OBJECTS)
	g++ -std=c++11 -o $@ $^ $(LDFLAGS)
 
# compile
%.o : %.cpp
	g++ -std=c++11 $(CFLAGS) -c -o $@ $<
 
# cleaning
clean:
	rm -f $(TARGET) $(OBJECTS)
