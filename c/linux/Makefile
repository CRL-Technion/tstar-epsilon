CXX      := -g++
CXXFLAGS := -Wno-unused-variable -fPIC -Wall -Wextra -O3
LDFLAGS  := 
LIBS 	 := -lVarSpeed2WindWoIO
OBJ_DIR  := obj
TARGET   := libtstar.so
INCLUDE  := -I../include/
BIN      := ../bin
SRC      := $(wildcard ../source/*.cpp)


OBJECTS := $(SRC:%.cpp=$(OBJ_DIR)/%.o)

all: build $(TARGET)
	cp $(TARGET) $(BIN)/$(TARGET)
	
$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<

$(TARGET): $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) --shared $(INCLUDE) -o $(TARGET) $(OBJECTS) $(LDFLAGS) $(LIBS)

.PHONY: all build clean

build:
	@mkdir -p $(OBJ_DIR)

clean:
	-@rm -rvf $(TARGET)
	-@rm -rvf $(OBJ_DIR)/*
	-@rm -rvf source/*
   
