PROJECT = Communication.a
OBJECTS = CanAdv.o CommImpl.o CommPar.o \
		CommPrint.o Communication.o DummyImpl.o
CFLAGS = -Wall -pedantic

all: $(PROJECT)
.cpp.o:
	g++ -c $(CFLAGS) $<
#create static library archive from OBJECTS
$(PROJECT): $(OBJECTS)
	ar rcso $(PROJECT) $(OBJECTS)
