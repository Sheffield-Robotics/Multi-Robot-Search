# Export some shell variables to be accessed by subscripts
# 
# The basedirectory of the graph_clear libraries includes various modules
# Each module has its own subdirectory to keep the source files
# All graph-clear headers are in ./include
# Each module is compiled in its directory linking to ../include
# 
# These directory specifiers are relative from the compiling directory 
# ../ is here for these directories
export COMP_TO_ROOT = ../
export GLOBAL_LIB_DIR = ../../lib/
export GLOBAL_DIR = ../../

# Library & Includes
export INCLUDES = -I$(COMP_TO_ROOT)include -I/usr/local/include -I/usr/local/include/opencv -I$(GLOBAL_DIR)br/include
export LIBRARIES_DIR = -L$(GLOBAL_LIB_DIR) -L/usr/local/lib 
# -lml
export LIBRARIES = -lcxcore -lcv -lhighgui -lcvaux -lgslwrap -lgsl -lgslcblas -lm -lsgbase -lsgmaps -lstdc++ -lconfig -lconfig++
export LFLAGS = $(LIBRARIES_DIR) $(LIBRARIES)

# Basic Options: compiler and compiler flags
export CC = g++
export CFLAGS_BIN = -Wall $(INCLUDES)
export CFLAGS = $(CFLAGS_BIN) -c -g -frounding-math

# These are all current components of the graph_clear library
COMPONENT_DIRS = base maps strategies

# Parse through all items listed in COMPONENT_DIRS and enter the directory 
all:
	@for dir in $(COMPONENT_DIRS); do\
		cd $$dir; \
		touch .depend; \
		$(MAKE) -f $(COMP_TO_ROOT)makefile.$$dir;\
		cd ..; \
	done

clean:
	$(RM) core *.o *~ lib/*.* bin/*test*
	@for dir in $(COMPONENT_DIRS); do \
		$(RM) -f $$dir/*.o; \
	done


