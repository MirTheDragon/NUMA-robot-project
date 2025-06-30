CC = gcc
CXX = g++
CFLAGS = -Wall -O2
CXXFLAGS = -Wall -O2

TARGET = ServoController_test

OBJS = PCA9685.o ServoController.o ServoController_test.o

all: $(TARGET)

PCA9685.o: PCA9685.c PCA9685.h
	$(CC) $(CFLAGS) -c PCA9685.c -o PCA9685.o

ServoController.o: ServoController.cpp ServoController.hpp PCA9685.h
	$(CXX) $(CXXFLAGS) -c ServoController.cpp -o ServoController.o

ServoController_test.o: ServoController_test.cpp ServoController.hpp PCA9685.h
	$(CXX) $(CXXFLAGS) -c ServoController_test.cpp -o ServoController_test.o

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) -lrt -lpthread

clean:
	rm -f $(OBJS) $(TARGET)
