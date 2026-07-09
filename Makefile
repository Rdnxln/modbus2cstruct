
SRCS = \
  modbus_expr.c \
  modbus_reqs.c \


OBJS =$(SRCS:.c=.o)
HDRS =$(SRCS:.c=.h)

OPTS =-Wall -Wextra -Wpedantic -std=c11

MODBUS_CFLAGS=`pkg-config libmodbus --cflags --with-path=./local_usr/lib/pkgconfig/`
MODBUS_LDLIBS=`pkg-config libmodbus --libs   --with-path=./local_usr/lib/pkgconfig/`

all:    solution

solution: main.c $(OBJS)
	gcc -g -ggdb $(OPTS) $(MODBUS_CFLAGS) ./main.c -o ./solution $(OBJS) -lm $(MODBUS_LDLIBS)

$(OBJS):    $(SRCS)
	gcc $(MODBUS_CFLAGS) -c $^ $(MODBUS_LDLIBS)

.PHONY: clean

clean:
	rm -f ./solution $(OBJS)
