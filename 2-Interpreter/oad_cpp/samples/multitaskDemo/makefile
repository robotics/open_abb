#Makefile for oad multitask Demo program

CXX      = g++
CPPFLAGS = -O2 -Wall -ansi
ODIR     = bin

.PHONY: clean


all: $(ODIR)/multitaskDemo 

$(ODIR)/multitaskDemo: $(OBJS) src/multitaskDemo.cpp
	$(CXX) $(CPPFLAGS) -o $(ODIR)/multitaskDemo src/multitaskDemo.cpp -load -lpthread -L../../lib -I../../include

clean:
	rm -f $(ODIR)/multitaskDemo
