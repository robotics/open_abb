#Makefile for oad library

PROJECT  = oad
CXX      = g++
CPPFLAGS = -O2 -Wall -ansi
LIB      = lib/lib$(PROJECT).a
SDIR     = samples
OBJ    = oad.o


#I think we can probably remove this
# Add flags if system is 64-bit
#LBITS := $(shell getconf LONG_BIT)
#ifeq ($(LBITS),64)
#CPPFLAGS += -arch x86_64
#endif

.PHONY: clean

all: $(LIB) include clean doc

$(LIB): $(OBJ)
	ar -r $(LIB) $(OBJ)

$(OBJ): src/oad.h
	$(CXX) $(CPPFLAGS) -c -o $(OBJ) src/oad.cpp	

include: $(LIB)
	cp src/oad.h include/oad.h

clean:
	rm -f $(OBJ)
	
doc: $(LIB)
	doxygen doc/DoxyfileHTML


#sample programs
samples: $(LIB) $(SDIR)/basicDemo/basicDemo $(SDIR)/multitaskDemo/multitaskDemo clean

$(SDIR)/basicDemo/basicDemo:
	make -C $(SDIR)/basicDemo

$(SDIR)/multitaskDemo/multitaskDemo:
	make -C $(SDIR)/multitaskDemo
