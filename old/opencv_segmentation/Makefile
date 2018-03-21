TARGET = test

CFLAGS = -Ofast -Wall -std=c++11 -Dsoc_cv_av -I/usr/local/include/hwlib/include -I/usr/local/include/hwlib/include/soc_cv_av `pkg-config --cflags opencv`
LDFLAGS = -Ofast -Wall -std=c++11 -Dsoc_cv_av `pkg-config --libs opencv`
CC = g++

all: $(TARGET)

$(TARGET): $(TARGET).o
	$(CC) $(LDFLAGS) -o $@ $^ `pkg-config --libs opencv` -lpthread

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(TARGET) *.a *.o *~
