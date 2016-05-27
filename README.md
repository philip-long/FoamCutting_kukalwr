# FoamCutting_kukalwr

Programs used during my thesis for cutting soft objects using 
collaborative robots.

Although the code is not at all optimized, the functions are
useful and mathematically robust. In addition to that
the visual servoing controllers worked very well and some 
of the calibration stuff has served me well.

in the make folder FastResearchInterfaceTest, Makefile

$(OBJ_DIR)/%.$(OBJECT_FILE_EXT): $(SRC_DIR)/FastResearchInterfaceTest/%.cpp
	@echo $(LINE1)
	$(CC) $(VISP_CFLAGS) $< -o $@
# the orginal one is $(CC) $< -o $@



$(EXES): $(OBJS)
	@echo $(LINE1)
	$(CL) -o $@ $^ -lm  -lFastResearchInterfaceLibrary -lTypeIRML -lrt $(VISP_LDFLAGS)
# the orginal one is $(CL) -o $@ $^ -lm  -lFastResearchInterfaceLibrary -lTypeIRML -lrt
