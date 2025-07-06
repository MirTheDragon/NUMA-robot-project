CC       = gcc
CXX      = g++
CFLAGS   = -Wall -O2
CXXFLAGS = -Wall -O2 -I/usr/include/libevdev-1.0
LDFLAGS  = -lrt -lpthread -lSDL2 -lSDL2_gfx -levdev

# Define all targets and their source files here
TARGETS = main ServoController_test RobotController_test PCA9685_test ServoController_servotester main_walkcycletest

# Sources for each target
SRCS_main                  = main.cpp face_renderer.cpp gamepad.cpp PathPlanner.cpp RobotController.cpp RobotBody.cpp ServoController.cpp ThreePointLeg.cpp PCA9685.c
SRCS_main_walkcycletest    = main_walkcycletest.cpp face_renderer.cpp gamepad.cpp PathPlanner.cpp RobotController.cpp RobotBody.cpp ServoController.cpp ThreePointLeg.cpp PCA9685.c
SRCS_ServoController_test  = PCA9685.c ServoController.cpp ServoController_test.cpp
SRCS_RobotController_test  = PCA9685.c ServoController.cpp ThreePointLeg.cpp RobotBody.cpp RobotController.cpp RobotController_test.cpp
SRCS_PCA9685_test          = PCA9685.c PCA9685_test.cpp
SRCS_ServoController_servotester = PCA9685.c ServoController.cpp ServoController_servotester.cpp

# Pattern to convert source files to objects
define OBJ_FILES
$(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$($(1))))
endef

OBJS_main                  = $(call OBJ_FILES,SRCS_main)
OBJS_main_walkcycletest    = $(call OBJ_FILES,SRCS_main_walkcycletest)
OBJS_ServoController_test  = $(call OBJ_FILES,SRCS_ServoController_test)
OBJS_RobotController_test  = $(call OBJ_FILES,SRCS_RobotController_test)
OBJS_PCA9685_test          = $(call OBJ_FILES,SRCS_PCA9685_test)
OBJS_ServoController_servotester = $(call OBJ_FILES,SRCS_ServoController_servotester)

.PHONY: all clean

all: $(TARGETS)

# Compile .cpp to .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile .c to .o
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link targets
main: $(OBJS_main)
	$(CXX) $^ -o $@ $(LDFLAGS)

main_walkcycletest: $(OBJS_main_walkcycletest)
	$(CXX) $^ -o $@ $(LDFLAGS)

ServoController_test: $(OBJS_ServoController_test)
	$(CXX) $^ -o $@ $(LDFLAGS)

RobotController_test: $(OBJS_RobotController_test)
	$(CXX) $^ -o $@ $(LDFLAGS)

PCA9685_test: $(OBJS_PCA9685_test)
	$(CXX) $^ -o $@ $(LDFLAGS)

ServoController_servotester: $(OBJS_ServoController_servotester)
	$(CXX) $^ -o $@ $(LDFLAGS)

clean:
	rm -f *.o $(TARGETS)
