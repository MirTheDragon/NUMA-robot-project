CC = gcc
CXX = g++
CFLAGS = -Wall -O2
CXXFLAGS = -Wall -O2

TARGET = PCA9685_test
OBJS = PCA9685.o PCA9685_test.o

all: $(TARGET)

PCA9685.o: PCA9685.c PCA9685.h
	$(CC) $(CFLAGS) -c PCA9685.c

PCA9685_test.o: PCA9685_test.cpp PCA9685.h
	$(CXX) $(CXXFLAGS) -c PCA9685_test.cpp

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) -lrt -lpthread

clean:
	rm -f $(OBJS) $(TARGET)
