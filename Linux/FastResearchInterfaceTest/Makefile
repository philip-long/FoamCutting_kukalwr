# Makefile
# 
# Stanford University
# Department of Computer Science
# Artificial Intelligence Laboratory
# Gates Computer Science Building 1A
# 353 Serra Mall
# Stanford, CA 94305-9010
# USA
#
# http://cs.stanford.edu/groups/manips
#
# Author : Torsten Kroeger, tkr@stanford.edu
# Date   : November 2011
#
#

all:

include ../Makefile.global

export

# ********************************************************************************
# ********************************************************************************
# LINUX
# ********************************************************************************
# ********************************************************************************

ifeq ($(TARGET_OS),__LINUX__)

include ../TargetDependencies.global

export

SRCS         := $(shell find $(SRC_DIR)/FastResearchInterfaceTest -name '*.cpp')
OBJS         := $(patsubst $(SRC_DIR)/FastResearchInterfaceTest/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))
EXES         := $(EXE_DIR)/FastResearchInterfaceTest

# ---------- Objects ----------
#this is VISP include path and lib path  /home/kuka/ViSP/visp
CXX=g++
VISP_BUILD_DIR = /usr
VISP_CFLAGS = `$(VISP_BUILD_DIR)/bin/visp-config --cflags`
VISP_LDFLAGS =`$(VISP_BUILD_DIR)/bin/visp-config --libs`
OPEN_CFLAGS= `pkg-config --cflags opencv gtk+-2.0` -I. 
OPEN_LDFLAGS= `pkg-config --libs opencv gtk+-2.0` -L. -lblob 


$(OBJ_DIR)/%.$(OBJECT_FILE_EXT): $(SRC_DIR)/FastResearchInterfaceTest/%.cpp
	@echo $(LINE1)
	$(CC) $(VISP_CFLAGS) $(OPEN_CFLAGS)$< -o $@
# visp only $(CC) $(VISP_CFLAGS) $< -o $@
# the orginal one is $(CC) $< -o $@

# -------- OutputFiles --------

$(EXES): $(OBJS)
	@echo $(LINE1)
	$(CL) -o $@ $^ -lm  -lFastResearchInterfaceLibrary -lTypeIRML $(OPEN_LDFLAGS) -lrt $(VISP_LDFLAGS)
#	visp only $(CL) -o $@ $^ -lm  -lFastResearchInterfaceLibrary -lTypeIRML -lrt $(VISP_LDFLAGS)
# the orginal one is $(CL) -o $@ $^ -lm  -lFastResearchInterfaceLibrary -lTypeIRML -lrt

# ****************************************************************
# Entry point for 'do_all'
# ****************************************************************

do_all: $(EXES)
	@echo Built directory contents
	@echo $(LINE1)
	@echo Operating system: $(TARGET_OS) [$(VARIANT)]
	

# ****************************************************************
# Entry point for 'do_clean'
# ****************************************************************

do_clean:
	@echo $(LINE1)
	@echo Operating system: $(TARGET_OS) [$(VARIANT)]
	rm -f $(OBJS)
	rm -f $(EXES)

include ../ExternalTargets.global

endif

	

