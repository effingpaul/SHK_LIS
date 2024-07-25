BASE = ../../../../../git/botop/rai
BASE2 = ../../../../../git/botop

DEPEND = KOMO Core Geo Kin Gui Optim

INCL = -I./include -I.

LIBS := $(shell pkg-config --libs opencv4)
LIBS += -lpthread -I/usr/include/opencv4 -lplibjs

OBJS = main.o src/CameraRecorder.o src/gamepad.o  #src/jsLinux.o
CPPFLAGS += -I/usr/local/include/opencv4
CXXFLAGS += -I/usr/local/include/opencv4


include $(BASE)/_make/generic.mk
