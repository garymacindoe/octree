CXXFLAGS = -std=c++11 -pedantic -Wall -Wextra -Werror

.PHONY: all clean

all: test_map

test: test_map
	./test_map

clean:
	$(RM) test_map

test_map: test_map.cpp
