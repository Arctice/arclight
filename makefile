flags = -std=c++2a -Wall -Wextra -Wpedantic -march=native -fopenmp=libiomp5
libs = -lomp5 -pthread -latomic -lfmt -lsfml-window -lsfml-graphics -lsfml-system -lopenblas

src = arcl.cpp ply.cpp
objs=$(src:.cpp=.o)

%.o : %.cpp
	clang++ ${flags} -c -o $@ $<

build: $(objs)
	clang++ ${flags} -fuse-ld=lld -o arcl ${objs} ${libs}

clean:
	rm *.o arcl

dev: flags += -O3
dev: build

debug: flags += -O1 -fsanitize=undefined,address -g -fno-omit-frame-pointer
debug: build
