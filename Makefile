CXXFLAGS   = -O2 -g3 -Wall -Wextra -fmessage-length=0 -fstack-protector-all -std=c++11
LDFLAGS    =
EXECUTABLE = engine
EXTENSION  = cc
SOURCES    = $(basename $(shell find . -name '*.$(EXTENSION)'))

.PHONY: all
all: $(EXECUTABLE)

$(EXECUTABLE): $(addsuffix .o,$(SOURCES))
	$(CXX) $(LDFLAGS) $^ -o $@

.PHONY: clean
clean:
	find . -name '*.o' -delete
	find . -name '*.d' -delete
	find . -name '*~'  -delete
	find . -name 'engine' -delete
	find . -name '*.bmp' -delete
	
.PHONY: tar
tar:
	make clean
	tar -cvzf s0160651.tar.gz Makefile src/* README.txt