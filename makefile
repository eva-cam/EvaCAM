CC=g++

ROOT_DIR=$(shell pwd)

BIN=Eva-CAM

OBJ = obj

CUR_SOURCE=${wildcard *.cpp}

CUR_OBJS=${patsubst %.cpp, %.o, $(CUR_SOURCE)}



all:$(CUR_OBJS)
	$(CC) -o $(BIN) $^

$(CUR_OBJS):%.o:%.cpp
	$(CC) -c $^ -o $@


.PHONY:clean
clean:
	@-rm $(ROOT_DIR)/*.o
	@-rm $(ROOT_DIR)/$(BIN)
