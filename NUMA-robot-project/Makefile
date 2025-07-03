CC        = gcc
CXX       = g++
CFLAGS    = -Wall -O2
CXXFLAGS  = -Wall -O2 -I/usr/include/libevdev-1.0

# Face main target (robot controller + face renderer + gamepad)
TARGET_FACE = main
OBJS_FACE = main.o face_renderer.o gamepad.o

# ServoController_test target
TARGET1   = ServoController_test
OBJS1     = PCA9685.o ServoController.o ServoController_test.o

# RobotController_test target
TARGET2   = RobotController_test
OBJS2     = PCA9685.o ServoController.o ThreePointLeg.o RobotBody.o RobotController.o RobotController_test.o

# PCA9685_test target
TARGET3   = PCA9685_test
OBJS3     = PCA9685.o PCA9685_test.o

# ServoController_servotester target
TARGET4   = ServoController_servotester
OBJS4     = PCA9685.o ServoController.o ServoController_servotester.o

all: $(TARGET1) $(TARGET2) $(TARGET3) $(TARGET4) $(TARGET_FACE)


face_renderer.o: face_renderer.cpp face_renderer.hpp
	$(CXX) $(CXXFLAGS) -c face_renderer.cpp -o face_renderer.o

gamepad.o: gamepad.cpp gamepad.hpp
	$(CXX) $(CXXFLAGS) -c gamepad.cpp -o gamepad.o

main.o: main.cpp gamepad.hpp face_renderer.hpp
	$(CXX) $(CXXFLAGS) -c main.cpp -o main.o

PCA9685.o: PCA9685.c PCA9685.h
	$(CC)   $(CFLAGS) -c PCA9685.c   -o PCA9685.o
	
PCA9685_test.o: PCA9685_test.cpp PCA9685.h
	$(CXX) $(CXXFLAGS) -c PCA9685_test.cpp -o PCA9685_test.o

ServoController.o: ServoController.cpp ServoController.hpp PCA9685.h
	$(CXX) $(CXXFLAGS) -c ServoController.cpp -o ServoController.o

ServoController_test.o: ServoController_test.cpp ServoController.hpp PCA9685.h
	$(CXX) $(CXXFLAGS) -c ServoController_test.cpp -o ServoController_test.o
	
ServoController_servotester.o: ServoController_servotester.cpp ServoController.hpp PCA9685.h
	$(CXX) $(CXXFLAGS) -c ServoController_servotester.cpp -o ServoController_servotester.o

ThreePointLeg.o: ThreePointLeg.cpp ThreePointLeg.hpp
	$(CXX) $(CXXFLAGS) -c ThreePointLeg.cpp   -o ThreePointLeg.o

RobotBody.o: RobotBody.cpp RobotBody.hpp
	$(CXX) $(CXXFLAGS) -c RobotBody.cpp       -o RobotBody.o

RobotController.o: RobotController.cpp RobotController.hpp ServoController.hpp ThreePointLeg.hpp RobotBody.hpp
	$(CXX) $(CXXFLAGS) -c RobotController.cpp -o RobotController.o

RobotController_test.o: RobotController_test.cpp RobotController.hpp
	$(CXX) $(CXXFLAGS) -c RobotController_test.cpp -o RobotController_test.o

$(TARGET1): $(OBJS1)
	$(CXX) $(OBJS1) -o $(TARGET1) -lrt -lpthread

$(TARGET2): $(OBJS2)
	$(CXX) $(OBJS2) -o $(TARGET2) -lrt -lpthread

$(TARGET3): $(OBJS3)
	$(CXX) $(OBJS3) -o $(TARGET3) -lrt -lpthread
	
$(TARGET4): $(OBJS4)
	$(CXX) $(OBJS4) -o $(TARGET4) -lrt -lpthread
	
$(TARGET_FACE): $(OBJS_FACE)
	$(CXX) $(OBJS_FACE) -o $(TARGET_FACE) -lrt -lpthread -lSDL2 -lSDL2_gfx -levdev


clean:
	rm -f $(OBJS1) $(OBJS2) $(OBJS3) $(OBJS4) $(TARGET1) $(TARGET2) $(TARGET3) $(TARGET4) $(TARGET_FACE)

