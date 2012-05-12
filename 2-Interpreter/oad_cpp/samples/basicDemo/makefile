#Makefile for oad basic demo program

CXX      = g++
CPPFLAGS = -O2 -Wall -ansi
ODIR     = bin

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
# If mac, SFML is linked differently
SFML_LIBS = -framework sfml-network -framework sfml-system
else
# Link libraries normally in linux
SFML_LIBS = -lsfml-network -lsfml-system
endif

.PHONY: clean


all: $(ODIR)/basicDemo 

$(ODIR)/basicDemo: $(OBJS) src/basicDemo.cpp
	$(CXX) $(CPPFLAGS) -o $(ODIR)/basicDemo src/basicDemo.cpp $(SFML_LIBS) -load -L../../lib -I../../include

clean:
	rm -f $(ODIR)/basicDemo
