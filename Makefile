# Makefile for Point Cloud Processing Application

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = -I/usr/include/pcl-1.12 -I/usr/include/eigen3 -I/usr/include/vtk-9.1
LIBS = -lpcl_common -lpcl_io -lpcl_filters -lpcl_segmentation -lpcl_visualization \
       -lpcl_search -lpcl_features -lpcl_surface -lvtkCommonCore -lvtkRenderingCore \
       -lvtkRenderingOpenGL2 -lvtkInteractionStyle -lvtkCommonDataModel -lvtkFiltersCore \
       -lvtkCommonExecutionModel -lboost_system -lboost_filesystem -lboost_thread \
       -lboost_date_time -lboost_iostreams -lboost_serialization -pthread

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
	sudo apt-get install -y libpcl-dev libeigen3-dev libvtk9-dev

.PHONY: install-deps
)