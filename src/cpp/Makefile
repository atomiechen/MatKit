HEADERS := $(wildcard *.h *.hpp)
SOURCES := $(wildcard *.cpp)
OBJS := $(SOURCES:.cpp=.o)
BINARIES := main

CC := g++
STD := -std=c++11
LDFLAGS := -lserial -L. -lfftw3 -lm
INCLUDE := -Iinclude

.PHONY: all clean rebuild info

all: $(BINARIES)

$(BINARIES): $(OBJS)
	$(CC) $(STD) $(LDFLAGS) -o $@ $^

%.o: %.cpp $(HEADERS)
	$(CC) $(STD) $(INCLUDE) -c -o $@ $<

clean:
	rm -f $(OBJS)

veryclean: clean
	rm -f $(BINARIES)

rebuild: veryclean all

info:
	@echo CC: $(CC)
	@echo STD: $(STD)
	@echo LDFLAGS: $(LDFLAGS)
	@echo INCLUDE: $(INCLUDE)
	@echo HEADERS: $(HEADERS)
	@echo SOURCES: $(SOURCES)
	@echo OBJS: $(OBJS)
	@echo BINARIES: $(BINARIES)
