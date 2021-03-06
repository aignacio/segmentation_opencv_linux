TARGET = crios_seg

ARCH= arm
HWLIBS_ROOT = /usr/local/include/hwlib/include
ALT_DEVICE_FAMILY = soc_cv_av
CFLAGS = -g -Ofast -Wall -std=c++11 -D$(ALT_DEVICE_FAMILY) -I$(HWLIBS_ROOT) -I$(HWLIBS_ROOT)/soc_cv_av `pkg-config --cflags opencv`
LDFLAGS = -g -Ofast -Wall -std=c++11 -D$(ALT_DEVICE_FAMILY) `pkg-config --libs opencv`
CC = arm-linux-gnueabihf-g++

all: $(TARGET)

$(TARGET): $(TARGET).o
	$(CC) $(LDFLAGS) -o $@ $^ `pkg-config --libs opencv` -lpthread

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(TARGET) *.a *.o *~
