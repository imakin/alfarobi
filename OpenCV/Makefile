###############################################################
# Makefile OpenCV Raspberry
###############################################################

TARGET = makin

CXX 		= g++
CFLAGS		= -Wall
LDFLAGS		= -pipe -O3 
# -mfpu=neon -ftree-vectorize -mfloat-abi=hard 
CLIBS 		=  -lserial -lopencv_core -lopencv_highgui -lopencv_imgproc

all: $(TARGET)

clean:
	rm -f $(TARGET) core *~ *.so *.lo

$(TARGET): makin.cpp 
	$(CXX) $(CFLAGS) $(LDFLAGS) makin.cpp $(CLIBS) -o $(TARGET)
