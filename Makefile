SOURCE_DIR=src
BUILD_DIR=build
T3D_INST=$(shell realpath ./tiny3d)

MODEL_SCALE=32

include $(N64_INST)/include/n64.mk
include $(T3D_INST)/t3d.mk

MK_ASSET=$(N64_INST)/bin/mkasset

N64_CFLAGS += -std=gnu2x -O2 -DMODEL_SCALE=$(MODEL_SCALE)
# N64_ASSET_FLAGS += -c 2 -w 256

PROJECT_NAME=t3d_test

src = src/main.c

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
MESH_SOURCES += $(shell find assets/maps -type f -name '*.glb' | sort)

T3DMESHES := $(MESH_SOURCES:assets/%.glb=filesystem/%.t3dm)

# inverse Model Scale will be applied to model transforms in game before rendering
filesystem/%.t3dm: assets/%.glb
	@mkdir -p $(dir $@)
	@mkdir -p $(dir $(@:filesystem/%.t3dm=build/assets/%.t3dm))
	@echo "    [T3DMODEL] $@"
	$(T3D_GLTF_TO_3D) "$<" $@ --base-scale=$(MODEL_SCALE)
	$(N64_BINDIR)/mkasset -o $(dir $@) -w 256

#----------------
# Maps
#----------------

MAP_SOURCES := $(shell find assets/maps -type f -name '*.blend' | sort)

COLLISION_EXPORT_FILE := $(shell find tools/collision_export/ -type f -name '*.py' | sort)
BLENDER_4 := blender

COLLISION_MESHES := $(MAP_SOURCES:assets/maps/%.blend=filesystem/maps/%.cmsh)

filesystem/maps/%.cmsh: assets/maps/%.blend $(COLLISION_EXPORT_FILE)
	@mkdir -p $(dir $@)
	@mkdir -p $(dir $(@:filesystem/%.cmsh=build/assets/%.cmsh))
	@echo "    [COLL_MESH] $@"
	$(BLENDER_4) $< --background --python-exit-code 1 --python tools/collision_export/collision_export.py -- $(@:filesystem/maps/%.cmsh=build/assets/maps/%.cmsh) 1
	$(N64_BINDIR)/mkasset -o $(dir $@) -w 256 $(@:filesystem/maps/%.cmsh=build/assets/maps/%.cmsh)


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

filesystem/: $(SPRITES) $(T3DMESHES) $(FONTS) $(MATERIALS) $(COLLISION_MESHES)

$(BUILD_DIR)/$(PROJECT_NAME).dfs: filesystem/ $(SPRITES) $(T3DMESHES) $(FONTS) $(MATERIALS) $(COLLISION_MESHES)
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
