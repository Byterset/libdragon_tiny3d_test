SOURCE_DIR=src
BUILD_DIR=build
T3D_INST=$(shell realpath ../tiny3d)

include $(N64_INST)/include/n64.mk
include $(T3D_INST)/t3d.mk

N64_CFLAGS += -std=gnu2x -Og
# N64_ASSET_FLAGS += -c 2 -w 256

PROJECT_NAME=t3d_test

src = src/main.c

all: $(PROJECT_NAME).z64

#----------------
# Images
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

filesystem/models/%.t3dm: assets/models/%.glb
	@mkdir -p $(dir $@)
	@mkdir -p $(dir $(@:filesystem/models/%.t3dm=build/assets/models/%.t3dm))
	$(T3D_GLTF_TO_3D) "$<" $@
	$(N64_BINDIR)/mkasset -o $(dir $@) -w 256

#----------------
# Code
#----------------

SOURCES := $(shell find src/ -type f -name '*.c' | sort)
SOURCE_OBJS := $(SOURCES:src/%.c=$(BUILD_DIR)/%.o)

#----------------
# Filesystem & Linking
#----------------	

filesystem/: $(SPRITES) $(T3DMESHES) $(FONTS)

$(BUILD_DIR)/$(PROJECT_NAME).dfs: filesystem/ $(SPRITES) $(TMESHES) $(FONTS)
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