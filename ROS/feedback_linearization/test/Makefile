CCLNFLAGS = -lstdc++ -leng -lMatlabEngine -lMatlabDataArray -lmx -lmat -lm -lpthread -ldl -lPOLIMIControl -lpython2.7 -lmatio

CCLNDIRS = -L/usr/local/MATLAB/R2021a/bin/glnxa64 -L/usr/local/MATLAB/R2021a/extern/bin/glnxa64

CCOPT = -m64 -fexceptions -fPIC -fno-omit-frame-pointer -fno-strict-aliasing -pthread -O2 -fwrapv -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/local/MATLAB/R2021a/extern/include -I/usr/local/MATLAB/R2021a/simulink/include -I/usr/include/eigen3 -I/usr/include/python2.7 -I/usr/local/include/POLIMIcontrol -I../include

CCC = g++ -O0 -std=c++11

CC = gcc

fblin_schlacher.o:
	$(CC) -c $(CCFLAGS) ../src/fblin_schlacher.cpp -o fblin_schlacher.o

test_fblin_schlacher: fblin_schlacher.o test_fblin_schlacher.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_fblin_schlacher fblin_schlacher.o test_fblin_schlacher.o $(CCLNFLAGS)
test_fblin_schlacher.o: test_fblin_schlacher.cpp
	$(CC) -c $(CCFLAGS) test_fblin_schlacher.cpp -o test_fblin_schlacher.o

all: test_fblin_schlacher

clean:
	@rm -rf *~ *.o
	@rm -rf test_fblin_schlacher
