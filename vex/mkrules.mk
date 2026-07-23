# VEXcode mkrules.mk 2019_03_26_01

# Helpers for writing JSON with GNU Make's built-in file function.
comma := ,
json_escape = $(subst ",\",$(subst \,\\,$(1)))
object_for = $(BUILD)/CMakeFiles/$(PROJECT).dir/$(patsubst %.c,%.c.obj,$(patsubst %.cpp,%.cpp.obj,$(1)))
compiler_for = $(if $(filter %.c,$(1)),$(CC),$(CXX))
flags_for = $(if $(filter %.c,$(1)),$(CFLAGS),$(CXX_FLAGS))

# compile C files
$(BUILD)/CMakeFiles/$(PROJECT).dir/%.c.obj: %.c $(SRC_H)
	$(Q)$(MKDIR)
	$(ECHO) "CC  $<"
	$(Q)$(CC) $(CFLAGS) $(INC) -c -o $@ $<
	
# compile C++ files
$(BUILD)/CMakeFiles/$(PROJECT).dir/%.cpp.obj: %.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
	$(ECHO) "CXX $<"
	$(Q)$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<
	
# create executable 
$(BUILD)/$(PROJECT).elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LINK) $(LNK_FLAGS) -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@

# create binary 
$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	$(Q)$(OBJCOPY) -O binary $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).bin

# create archive
$(BUILD)/$(PROJECTLIB).a: $(OBJ)
	$(Q)$(ARCH) $(ARCH_FLAGS) $@ $^

# clean project
clean:
	$(info clean project)
	$(Q)$(CLEAN)

# Generate a compilation database for clangd and other C/C++ tooling.
.PHONY: compile_commands force_compile_commands
compile_commands: $(BUILD)/compile_commands.json

$(BUILD)/compile_commands.json: force_compile_commands
	$(Q)$(MKDIR)
	$(file >$@,[)
	$(foreach source,$(SRC_C),$(file >>$@,  {"directory":"$(call json_escape,$(CURDIR))","file":"$(call json_escape,$(abspath $(source)))","output":"$(call json_escape,$(abspath $(call object_for,$(source))))","command":"$(call json_escape,$(call compiler_for,$(source)) $(call flags_for,$(source)) $(INC) -c -o $(call object_for,$(source)) $(source))"}$(if $(filter $(source),$(lastword $(SRC_C))),,$(comma))))
	$(file >>$@,])
	$(ECHO) "Generated $@"
