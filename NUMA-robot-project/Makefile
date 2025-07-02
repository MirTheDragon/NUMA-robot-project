CC        = gcc
CXX       = g++
CFLAGS    = -Wall -O2
CXXFLAGS  = -Wall -O2

# ServoController_test target
TARGET1   = ServoController_test
OBJS1     = PCA9685.o ServoController.o ServoController_test.o

# RobotController_test target
TARGET2   = RobotController_test
OBJS2     = PCA9685.o ServoController.o ThreePointLeg.o RobotBody.o RobotController.o RobotController_test.o

all: $(TARGET1) $(TARGET2)

PCA9685.o: PCA9685.c PCA9685.h
	$(CC)   $(CFLAGS) -c PCA9685.c   -o PCA9685.o

ServoController.o: ServoController.cpp ServoController.hpp PCA9685.h
	$(CXX) $(CXXFLAGS) -c ServoController.cpp -o ServoController.o

ServoController_test.o: ServoController_test.cpp ServoController.hpp PCA9685.h
	$(CXX) $(CXXFLAGS) -c ServoController_test.cpp -o ServoController_test.o

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

clean:
	rm -f $(OBJS1) $(OBJS2) $(TARGET1) $(TARGET2)
