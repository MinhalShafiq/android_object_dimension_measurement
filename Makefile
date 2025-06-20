# Makefile for Point Cloud Processing Application

CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall
INCLUDES = -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -I/usr/include/vtk-7.1
LIBS = -lpcl_common -lpcl_io -lpcl_features -lpcl_filters -lpcl_segmentation \
       -lpcl_visualization -lpthread \
       -lvtkCommonCore-7.1 -lvtkCommonDataModel-7.1 -lvtkCommonTransforms-7.1 -lvtkCommonMath-7.1 \
       -lvtkRenderingCore-7.1 -lvtkRenderingOpenGL2-7.1 -lvtkInteractionStyle-7.1 -lvtkIOImage-7.1 \
       -lvtkCommonExecutionModel-7.1 -lboost_thread -lboost_chrono -lboost_system \
       -lvtkRenderingLOD-7.1 -lvtkFiltersSources-7.1 -lvtksys-7.1 \
       -lpcl_search -lpcl_surface -lpcl_kdtree

SRCDIR = src
OBJDIR = obj
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
TARGET = point_cloud_app

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@ $(LIBS)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OBJDIR):
	mkdir -p $(OBJDIR)

clean:
	rm -rf $(OBJDIR) $(TARGET)

install-deps:
	sudo apt-get update
	sudo apt-get install -y libpcl-dev libeigen3-dev libvtk7-dev

.PHONY: install-deps