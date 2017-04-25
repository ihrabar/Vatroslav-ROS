# Makefile for building the Devices layer

CPP = g++
INCDIR = ./
SRCDIR = ./
OBJDIR = ./obj/
BINDIR = ./bin/
#where to store library
LIBRARY = ../Bin/Devices.a
LIBCFLAGS = -Wall -pedantic
#from where to read librarys
LIBDIR = ../Bin/
CFLAGS = -O3 -I $(INCDIR) -I /usr/local/boost/boost_1_39_0 \
	 -I /usr/local/lib \
	-I /usr/include/boost	\
	-I /usr/lib	\
	-I /usr/lib/x86_64-linux-gnu	\
	-I /usr/include/loki -I ../ \
	-I /usr/local/loki-0.1.7/include -I ../ \
	-DVATROSLAV_UNO

LFLAGS = -lboost_system

DEV_OBJS = $(OBJDIR)MotorEPOS.o $(OBJDIR)flipper_test.o $(OBJDIR)LinAct.o \
		$(OBJDIR)Sensor.o  $(OBJDIR)Flipper.o $(OBJDIR)Kinematics.o \
		$(OBJDIR)WirelessVIV.o
LIBRARY_OBJS = $(LIBDIR)Communication.a 

all: $(BINDIR)flipper_test

#doc: $(INCDIR)*.hpp Doxyfile
#	@echo "Building Doxygen documentation..."
#	doxygen Doxyfile

clean:
	@echo "Cleaning up the Devices project..."
	rm -f $(OBJDIR)*.o
	rm -f $(BINDIR)flipper_test
	rm -f $(INCDIR)*~
	rm -f $(SRCDIR)*~
	
$(BINDIR)flipper_test: $(DEV_OBJS)
	@echo "Building flipper test executable..."
	$(CPP)  -o $(BINDIR)flipper_test $(DEV_OBJS) $(LIBRARY_OBJS) $(LFLAGS)

$(OBJDIR)MotorEPOS.o: $(INCDIR)MotorEPOS.hpp $(SRCDIR)MotorEPOS.cpp
	@echo "Compiling MotorEPOS..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)MotorEPOS.o $(SRCDIR)MotorEPOS.cpp

$(OBJDIR)LinAct.o: $(INCDIR)LinAct.hpp $(SRCDIR)LinAct.cpp
	@echo "Compiling LinAct..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)LinAct.o $(SRCDIR)LinAct.cpp

$(OBJDIR)Sensor.o: $(INCDIR)Sensor.hpp $(SRCDIR)Sensor.cpp
	@echo "Compiling Sensor..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)Sensor.o $(SRCDIR)Sensor.cpp

$(OBJDIR)Flipper.o: $(INCDIR)Flipper.hpp $(SRCDIR)Flipper.cpp
	@echo "Compiling Flipper..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)Flipper.o $(SRCDIR)Flipper.cpp

$(OBJDIR)Kinematics.o: $(INCDIR)Kinematics.hpp $(SRCDIR)Kinematics.cpp
	@echo "Compiling Kinematics..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)Kinematics.o $(SRCDIR)Kinematics.cpp

$(OBJDIR)WirelessVIV.o: $(INCDIR)WirelessVIV.hpp $(SRCDIR)WirelessVIV.cpp
	@echo "Compiling WirelessVIV..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)WirelessVIV.o $(SRCDIR)WirelessVIV.cpp

$(OBJDIR)flipper_test.o: $(INCDIR)*.hpp $(SRCDIR)flipper_test.cpp
	@echo "Compiling flipper_test..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)flipper_test.o $(SRCDIR)flipper_test.cpp

#creating library
all: $(LIBRARY)
.cpp.o:
	g++ -c $(LIBCFLAGS) $<
#create static library archive from OBJECTS
$(LIBRARY): $(DEV_OBJS)
	ar rcso $(LIBRARY) $(DEV_OBJS)
