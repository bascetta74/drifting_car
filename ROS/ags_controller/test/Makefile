CCLNFLAGS = -lstdc++ -lm -lpthread -ldl

CCLNDIRS = 

CCOPT = -m64 -fexceptions -fPIC -fno-omit-frame-pointer -fno-strict-aliasing -pthread -O2 -fwrapv -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I../include -I../../singletrack_sim/include

CCC = g++ -O0 -std=c++11

CC = gcc

singletrack_beta_velocity_ode.o:
	$(CC) -c $(CCFLAGS) ../../singletrack_sim/src/singletrack_beta_velocity_ode.cpp -o singletrack_beta_velocity_ode.o

velocity_sideslip_estimator.o:
	$(CC) -c $(CCFLAGS) ../src/velocity_sideslip_estimator.cpp -o velocity_sideslip_estimator.o
acceleration_sideslip_estimator.o:
	$(CC) -c $(CCFLAGS) ../src/acceleration_sideslip_estimator.cpp -o acceleration_sideslip_estimator.o

test_beta_estimator_velocity: singletrack_beta_velocity_ode.o velocity_sideslip_estimator.o test_beta_estimator_velocity.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_beta_estimator_velocity singletrack_beta_velocity_ode.o velocity_sideslip_estimator.o test_beta_estimator_velocity.o $(CCLNFLAGS)
test_beta_estimator_velocity.o: test_beta_estimator_velocity.cpp
	$(CC) -c $(CCFLAGS) test_beta_estimator_velocity.cpp -o test_beta_estimator_velocity.o

test_beta_estimator_acceleration: singletrack_beta_velocity_ode.o acceleration_sideslip_estimator.o test_beta_estimator_acceleration.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_beta_estimator_acceleration singletrack_beta_velocity_ode.o acceleration_sideslip_estimator.o test_beta_estimator_acceleration.o $(CCLNFLAGS)
test_beta_estimator_acceleration.o: test_beta_estimator_acceleration.cpp
	$(CC) -c $(CCFLAGS) test_beta_estimator_acceleration.cpp -o test_beta_estimator_acceleration.o

all: test_beta_estimator_velocity test_beta_estimator_acceleration

clean:
	@rm -rf *~ *.o
	@rm -rf test_beta_estimator_velocity test_beta_estimator_acceleration

