# Makefile for bulding the Communication layer

CPP = g++
INCDIR = ./
SRCDIR = ./
OBJDIR = ./obj/
BINDIR = ./bin/
LIBRARY = ../Bin/Communication.a
LIBCFLAGS = -Wall -pedantic
LIBDIR =  /usr/lib
CFLAGS = -O3 -I $(INCDIR) -I /usr/local/lib \
	-I /usr/include/boost	\
	-I /usr/lib	\
	-I /usr/lib/x86_64-linux-gnu	\
	-I /usr/include/loki -I ../ \
	-DVATROSLAV_UNO -DBOOST_SYSTEM_NO_DEPRECATED -lboost_system -lboost_thread

LFLAGS = -lboost_system -lboost_thread

COMM_OBJS = $(OBJDIR)DummyImpl.o $(OBJDIR)CommPrint.o $(OBJDIR)CommPar.o\
			  $(OBJDIR)CommImpl.o $(OBJDIR)Communication.o $(OBJDIR)comm_test.o $(OBJDIR)CanAdv.o\
			  $(OBJDIR)SerialBoost.o $(OBJDIR)TimeoutSerial.o

LIBRARY_OBJS = $(LIBDIR)*.so

all: $(BINDIR)comm_test

#doc: $(INCDIR)*.hpp Doxyfile
#	@echo "Building Doxygen documentation..."
#	doxygen Doxyfile

clean:
	@echo "Cleaning up the Communication project..."
	rm -f $(OBJDIR)*.o
	rm -f $(BINDIR)comm_test
	rm -f $(INCDIR)*~
	rm -f $(SRCDIR)*~

$(BINDIR)comm_test: $(COMM_OBJS)
	@echo "Building communication test executable..."
	$(CPP) -o $(BINDIR)comm_test $(COMM_OBJS) $(LFLAGS)

$(OBJDIR)CanAdv.o: $(INCDIR)CanAdv.hpp $(SRCDIR)CanAdv.cpp
	@echo "Compiling CanAdv..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)CanAdv.o $(SRCDIR)CanAdv.cpp

$(OBJDIR)DummyImpl.o: $(INCDIR)DummyImpl.hpp $(SRCDIR)DummyImpl.cpp
	@echo "Compiling DummyImpl..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)DummyImpl.o $(SRCDIR)DummyImpl.cpp

$(OBJDIR)CommPrint.o: $(INCDIR)CommPrint.hpp $(SRCDIR)CommPrint.cpp
	@echo "Compiling CommPrint..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)CommPrint.o $(SRCDIR)CommPrint.cpp

$(OBJDIR)CommPar.o: $(INCDIR)CommPar.hpp $(SRCDIR)CommPar.cpp
	@echo "Compiling CommPar Interface..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)CommPar.o $(SRCDIR)CommPar.cpp

$(OBJDIR)CommImpl.o: $(INCDIR)*.hpp $(SRCDIR)CommImpl.cpp
	@echo "Compiling CommImpl..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)CommImpl.o $(SRCDIR)CommImpl.cpp

$(OBJDIR)Communication.o: $(INCDIR)*.hpp $(SRCDIR)Communication.cpp
	@echo "Compiling Communication..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)Communication.o $(SRCDIR)Communication.cpp
	
$(OBJDIR)SerialBoost.o: $(INCDIR)*.hpp $(SRCDIR)SerialBoost.cpp
	@echo "Compiling SerialBoost..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)SerialBoost.o $(SRCDIR)SerialBoost.cpp

$(OBJDIR)TimeoutSerial.o: $(INCDIR)*.hpp $(SRCDIR)TimeoutSerial.cpp
	@echo "Compiling TimeoutSerial..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)TimeoutSerial.o $(SRCDIR)TimeoutSerial.cpp

$(OBJDIR)comm_test.o: $(INCDIR)*.hpp $(SRCDIR)comm_test.cpp
	@echo "Compiling comm_test..."
	$(CPP) $(CFLAGS) -c -o $(OBJDIR)comm_test.o $(SRCDIR)comm_test.cpp

all: $(LIBRARY)
.cpp.o:
	g++ -c $(LIBCFLAGS) $<
#create static library archive from OBJECTS
$(LIBRARY): $(COMM_OBJS)
	ar rcso $(LIBRARY) $(COMM_OBJS)
