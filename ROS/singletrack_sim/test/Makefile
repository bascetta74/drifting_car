CCLNFLAGS = -lstdc++ -lm -lpthread -ldl

CCLNDIRS = 

CCOPT = -m64 -fexceptions -fPIC -fno-omit-frame-pointer -fno-strict-aliasing -pthread -O2 -fwrapv -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I../include

CCC = g++ -O0 -std=c++11

CC = gcc

singletrack_beta_force_ode.o:
	$(CC) -c $(CCFLAGS) ../src/singletrack_beta_force_ode.cpp -o singletrack_beta_force_ode.o
singletrack_beta_velocity_ode.o:
	$(CC) -c $(CCFLAGS) ../src/singletrack_beta_velocity_ode.cpp -o singletrack_beta_velocity_ode.o
singletrack_vxvy_force_ode.o:
	$(CC) -c $(CCFLAGS) ../src/singletrack_vxvy_force_ode.cpp -o singletrack_vxvy_force_ode.o
singletrack_vxvy_velocity_ode.o:
	$(CC) -c $(CCFLAGS) ../src/singletrack_vxvy_velocity_ode.cpp -o singletrack_vxvy_velocity_ode.o

test_singletrack_beta_force_ode: singletrack_beta_force_ode.o test_singletrack_beta_force_ode.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_singletrack_beta_force_ode singletrack_beta_force_ode.o test_singletrack_beta_force_ode.o $(CCLNFLAGS)
test_singletrack_beta_force_ode.o: test_singletrack_beta_force_ode.cpp
	$(CC) -c $(CCFLAGS) test_singletrack_beta_force_ode.cpp -o test_singletrack_beta_force_ode.o

test_singletrack_beta_velocity_ode: singletrack_beta_velocity_ode.o test_singletrack_beta_velocity_ode.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_singletrack_beta_velocity_ode singletrack_beta_velocity_ode.o test_singletrack_beta_velocity_ode.o $(CCLNFLAGS)
test_singletrack_beta_velocity_ode.o: test_singletrack_beta_velocity_ode.cpp
	$(CC) -c $(CCFLAGS) test_singletrack_beta_velocity_ode.cpp -o test_singletrack_beta_velocity_ode.o

test_singletrack_vxvy_force_ode: singletrack_vxvy_force_ode.o test_singletrack_vxvy_force_ode.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_singletrack_vxvy_force_ode singletrack_vxvy_force_ode.o test_singletrack_vxvy_force_ode.o $(CCLNFLAGS)
test_singletrack_vxvy_force_ode.o: test_singletrack_vxvy_force_ode.cpp
	$(CC) -c $(CCFLAGS) test_singletrack_vxvy_force_ode.cpp -o test_singletrack_vxvy_force_ode.o

test_singletrack_vxvy_velocity_ode: singletrack_vxvy_velocity_ode.o test_singletrack_vxvy_velocity_ode.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_singletrack_vxvy_velocity_ode singletrack_vxvy_velocity_ode.o test_singletrack_vxvy_velocity_ode.o $(CCLNFLAGS)
test_singletrack_vxvy_velocity_ode.o: test_singletrack_vxvy_velocity_ode.cpp
	$(CC) -c $(CCFLAGS) test_singletrack_vxvy_velocity_ode.cpp -o test_singletrack_vxvy_velocity_ode.o

all: test_singletrack_beta_force_ode test_singletrack_beta_velocity_ode test_singletrack_vxvy_force_ode test_singletrack_vxvy_velocity_ode

clean:
	@rm -rf *~ *.o
	@rm -rf test_singletrack_beta_force_ode test_singletrack_beta_velocity_ode test_singletrack_vxvy_force_ode test_singletrack_vxvy_velocity_ode
