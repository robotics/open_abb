#Makefile for oad basic demo program

CXX      = g++
CPPFLAGS = -O2 -Wall -ansi
ODIR     = bin


.PHONY: clean


all: $(ODIR)/basicDemo 

$(ODIR)/basicDemo: $(OBJS) src/basicDemo.cpp
	$(CXX) $(CPPFLAGS) -o $(ODIR)/basicDemo src/basicDemo.cpp -load -L../../lib -I../../include

clean:
	rm -f $(ODIR)/basicDemo
