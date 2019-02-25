CXX=g++
CXXFLAGS=-std=c++11 -Wall -pedantic -O3
LIBS= -lGL -lGLU -lglut
all:	main
main:	main.o  
	$(CXX) -o main main.o $(CXXFLAGS) $(LIBS)
main.o:	main.cpp
	$(CXX) -o main.o -c main.cpp $(CXXFLAGS)
clean:
	rm -f *.o
