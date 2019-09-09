# the $< is the first item in the dependencies list
# $@ and $^ are the left and right sides of the :, respectively
# $* is the left side of the :, without suffix

CC = g++
CFLAGS = -pipe -O2 -Wall -Wextra -std=c++11
HEADERS = TestEnki.h model.h classifier.h parameters.h test.h SerialComm.h
ODIR = obj
_OBJ = test.o TestEnki.o model.o classifier.o SerialComm.o main.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
IDIR = -I. -I/usr/include -I/home/wawa/Turing_learning/enki
LDIR = -L. -L/home/wawa/Turing_learning/enki/enki -L/usr/lib -lm -lglut -lGLU -lGL -lenki -lgsl -lgslcblas
OUT = main

all: $(OUT)
$(OUT): $(OBJ)
	$(CC) -o $@ $^ $(LDIR)

$(ODIR)/%.o: %.cpp $(HEADERS)
	$(CC) -c -o $@ $< $(CFLAGS) $(IDIR)

clean:
	rm -rf $(ODIR)/*.o *.txt *~ $(OUT)
