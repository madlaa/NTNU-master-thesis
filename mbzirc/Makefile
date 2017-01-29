GCC = g++ -std=c++11 -I../include
CPPFLAGS = -std=c++11 -I../include  -I/usr/include/opencv2
CXXFLAGS += $(shell pkg-config opencv --cflags)
LIBS  += $(shell pkg-config opencv --libs)

OUT = start

all	:mbzirc.o \
         ../kinematics/ur_kin.o \
         ../ur_modern_driver-master/src/ur_driver.o \
	 ../ur_modern_driver-master/src/robot_state.o \
	 ../ur_modern_driver-master/src/do_output.o \
	 ../ur_modern_driver-master/src/robot_state_RT.o \
	 ../ur_modern_driver-master/src/ur_communication.o \
	 ../ur_modern_driver-master/src/ur_realtime_communication.o \
	 ../vision/vision.o \
	 ../force/force.o
	 $(GCC)  -o $(OUT) $^ $(LIBS) -lpthread -lm -lgsl -lgslcblas
	 
clean : 
	rm -f *.o $(OUT)