CXXFLAGS=-Wall -std=c++0x -march=native -O3 -DGPU_FRIENDLY
EXECUTABLE=cnvm
LDFLAGS=
OBJ=cnvm.o main.o

$(EXECUTABLE): $(OBJ)
	$(CXX) -o $@ $(OBJ) $(LDFLAGS)

cnvm.o: cnvm.cpp cnvm.h
main.o: main.cpp

clean:
	rm -f $(EXECUTABLE) $(OBJ)
