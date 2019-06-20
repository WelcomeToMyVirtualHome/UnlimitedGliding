CXX=g++
CXXFLAGS=-std=c++0x -Wall -pedantic -O3 -pthread
all:	main	clean
main:	main.o  
	$(CXX) -o main main.o $(CXXFLAGS)
main.o:	main.cpp
	$(CXX) -o main.o -c main.cpp $(CXXFLAGS)
clean:
	rm -f *.o
