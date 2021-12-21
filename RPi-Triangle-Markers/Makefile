CC=g++
CFLAGS=  -I/usr/local/include/opencv4 -L/usr/local/lib -g -Wall -pedantic -g
CLIBS= -lopencv_highgui -lopencv_aruco -lopencv_imgcodecs -lopencv_core -lopencv_videoio -lopencv_calib3d -lopencv_imgproc -lpthread
TARGET= main
all: $(TARGET)

$(TARGET): $(TARGET).cc
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).cc  $(CLIBS)

debug:
	$(CC) -debug -Wall $(CFLAGS) -o $(TARGET) $(TARGET).cc  $(CLIBS)


clean:
	$(RM) $(TARGET)
