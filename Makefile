SOURCE_DIR=src
BUILD_DIR=build
T3D_INST=$(shell realpath ./tiny3d)

include $(N64_INST)/include/n64.mk
include $(T3D_INST)/t3d.mk

MK_ASSET=$(N64_INST)/bin/mkasset

N64_CFLAGS += -std=gnu2x -Og
# N64_ASSET_FLAGS += -c 2 -w 256

PROJECT_NAME=t3d_test

src = src/main.c

LDFLAGS += -lraylib

all: $(PROJECT_NAME).z64

#----------------
# Images / Sprites
#----------------

PNG_RGBA16 := $(shell find assets/ -type f -name '*.png' | sort)

MKSPRITE_FLAGS ?=

SPRITES := $(PNG_RGBA16:assets/%.png=filesystem/%.sprite)

filesystem/%.sprite: assets/%.png
	@mkdir -p $(dir $@)
	@echo "    [SPRITE] $@"
	$(N64_MKSPRITE) -f ${shell jq -r .format ${<:%.png=%.json} || echo AUTO} --compress -o "$(dir $@)" "$<"

#----------------
# Fonts
#----------------

FONT_SOURCES := $(shell find assets/ -type f -name '*.ttf' | sort)

FONTS := $(FONT_SOURCES:assets/%.ttf=filesystem/%.font64)

MKFONT_FLAGS ?=

filesystem/%.font64: assets/%.ttf
	@mkdir -p $(dir $@)
	@echo "    [FONT] $@"
	@$(N64_MKFONT) $(MKFONT_FLAGS) -o $(dir $@) "$<"


#----------------
# 3D Models
#----------------

MESH_SOURCES := $(shell find assets/models -type f -name '*.glb' | sort)

T3DMESHES := $(MESH_SOURCES:assets/models/%.glb=filesystem/models/%.t3dm)

#base-scale option should be the same as SCENE_SCALE in the defs.h file. Default is 64, here we use 16 because of matrix conversion limitations
filesystem/models/%.t3dm: assets/models/%.glb
	@mkdir -p $(dir $@)
	@mkdir -p $(dir $(@:filesystem/models/%.t3dm=build/assets/models/%.t3dm))
	@echo "    [T3DMODEL] $@"
	$(T3D_GLTF_TO_3D) "$<" $@ --base-scale=16 
	$(N64_BINDIR)/mkasset -o $(dir $@) -w 256


#----------------
# Materials
#----------------

MATERIAL_SOURCES := $(shell find assets/ -type f -name '*.mat.json' | sort)

MATERIALS := $(MATERIAL_SOURCES:assets/%.mat.json=filesystem/%.mat)

filesystem/%.mat: assets/%.mat.json
	@mkdir -p $(dir $@)
	@mkdir -p $(dir $(@:filesystem/%.mat=build/assets/%.mat))
	@echo "    [MATERIAL] $@"
	python3 tools/mesh_export/material.py --default assets/materials/default.mat.json $< $(@:filesystem/%.mat=build/assets/%.mat)
	$(MK_ASSET) -o $(dir $@) -w 4 $(@:filesystem/%.mat=build/assets/%.mat)

#----------------
# Code
#----------------

SOURCES := $(shell find src/ -type f -name '*.c' | sort)
SOURCE_OBJS := $(SOURCES:src/%.c=$(BUILD_DIR)/%.o)

DEPS := $(SOURCE_OBJS:.o=.d)

# Include the dependency files, if they exist
-include $(DEPS)

#----------------
# Filesystem & Linking
#----------------	

filesystem/: $(SPRITES) $(T3DMESHES) $(FONTS) $(MATERIALS)

$(BUILD_DIR)/$(PROJECT_NAME).dfs: filesystem/ $(SPRITES) $(T3DMESHES) $(FONTS) $(MATERIALS)
$(BUILD_DIR)/$(PROJECT_NAME).elf: $(SOURCE_OBJS)

$(PROJECT_NAME).z64: N64_ROM_TITLE="Tiny3D Playground"
$(PROJECT_NAME).z64: $(BUILD_DIR)/$(PROJECT_NAME).dfs


clean:
	rm -rf $(BUILD_DIR) *.z64
	rm -rf filesystem

build_lib:
	rm -rf $(BUILD_DIR) *.z64
	make -C $(T3D_INST)
	make all

clean-fs:
	rm -rf filesystem/ $(BUILD_DIR)/$(PROJECT_NAME).dfs

-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean
