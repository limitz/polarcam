CC=cc
SRC=main.c
LIBS=-lgpiod
PROJ=event27

all:
	$(CC) $(SRC) $(LIBS) $(CFLAGS) -o $(PROJ)

clean:
	rm $(PROJ)
