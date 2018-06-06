
# For allowing png outputs during visualization, type this:
# make name_of_algo pngout=true
#
PNG_OUT := $(strip $(pngout))
DEBUG_MODE := $(strip $(debug))

DEFAULT_FLAGS := -lm
VIZUALIZER_FLAGS := -lGL -lGLU -lglut
PNGOUTPUT_FLAGS := -lIL -lILU -lILUT
ERROR_FLAGS := -Wall -Wextra
CANCEL_FLAGS := -Wno-unused-variable -Wno-unused-parameter -Wno-unused-but-set-variable -Wno-switch -Wno-comment -Wno-unused-but-set-parameter -Wunused-function

##############

GCC := gcc src/robotsim_main.c src/utilities/datastructs.c src/utilities/dynamics_utils.c src/utilities/math_utils.c src/utilities/file_utils.c src/utilities/param_utils.c src/sensors.c src/robotmodel.c src/colors.c src/vizualizer/objects_2d.c src/objects_menu.c src/utilities/arenas.c src/vizualizer/objects_3d.c src/stat.c src/dynspecviz.c src/utilities/output_utils.c src/utilities/debug_utils.c

# Setting up png output mode
ifeq ($(PNG_OUT), true)
 GCC += src/utilities/pngout_utils.c $(PNGOUTPUT_FLAGS) -DPNG_OUT
endif

# Debug mode for segfault detection
ifeq ($(DEBUG_MODE), true)
 GCC += -DDEBUG -rdynamic
endif

#GCC += $(DEFAULT_FLAGS) $(VIZUALIZER_FLAGS) $(ERROR_FLAGS) $(CANCEL_FLAGS) -o robotflocksim_main
GCC += $(DEFAULT_FLAGS) $(VIZUALIZER_FLAGS) $(ERROR_FLAGS) $(CANCEL_FLAGS) -o robotflocksim_main -pg

# SPP model for testing evolution algorithms
spp_evol:
	$(GCC) src/algo_spp_evol.c src/algo_spp_evol_gui.c src/algo_spp_evol_stat.c src/utilities/interactions.c src/utilities/obstacles.c
