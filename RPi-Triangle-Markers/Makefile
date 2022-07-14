CC=g++
CFLAGS=  -I/usr/local/include/opencv4 -L/usr/local/lib -g -Wall
CLIBS= -lopencv_highgui -lopencv_aruco -lopencv_imgcodecs -lopencv_core -lopencv_videoio -lopencv_calib3d -lopencv_imgproc -lpthread
JLIBS= -lSDL2 -lpthread

all: main joystick

main: main.cc
	$(CC) $(CFLAGS) -o main main.cc  $(CLIBS)
joystick: joystick.cc 
	$(CC) $(CFLAGS) -o joystick joystick.cc  $(JLIBS)


clean:
	$(RM) main joystick
