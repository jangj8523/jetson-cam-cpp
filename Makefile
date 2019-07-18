# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jaewoo/Desktop/project/jetson-inference

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jaewoo/Desktop/project/jetson-inference

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target install/strip
install/strip: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip

# Special rule for the target install/strip
install/strip/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/bin/ccmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target list_install_components
list_install_components:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Available install components are: \"Unspecified\""
.PHONY : list_install_components

# Special rule for the target list_install_components
list_install_components/fast: list_install_components

.PHONY : list_install_components/fast

# Special rule for the target install/local
install/local: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local

# Special rule for the target install/local
install/local/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local/fast

# Special rule for the target install
install: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install

# Special rule for the target install
install/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install/fast

# The main all target
all: cmake_check_build_system
	cd /home/jaewoo/Desktop/project/jetson-inference && $(CMAKE_COMMAND) -E cmake_progress_start /home/jaewoo/Desktop/project/jetson-inference/CMakeFiles /home/jaewoo/Desktop/project/jetson-inference/detectnet-camera/CMakeFiles/progress.marks
	cd /home/jaewoo/Desktop/project/jetson-inference && $(MAKE) -f CMakeFiles/Makefile2 detectnet-camera/all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/jaewoo/Desktop/project/jetson-inference/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	cd /home/jaewoo/Desktop/project/jetson-inference && $(MAKE) -f CMakeFiles/Makefile2 detectnet-camera/clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	cd /home/jaewoo/Desktop/project/jetson-inference && $(MAKE) -f CMakeFiles/Makefile2 detectnet-camera/preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	cd /home/jaewoo/Desktop/project/jetson-inference && $(MAKE) -f CMakeFiles/Makefile2 detectnet-camera/preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	cd /home/jaewoo/Desktop/project/jetson-inference && $(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

# Convenience name for target.
detectnet-camera/CMakeFiles/detectnet-camera.dir/rule:
	cd /home/jaewoo/Desktop/project/jetson-inference && $(MAKE) -f CMakeFiles/Makefile2 detectnet-camera/CMakeFiles/detectnet-camera.dir/rule
.PHONY : detectnet-camera/CMakeFiles/detectnet-camera.dir/rule

# Convenience name for target.
detectnet-camera: detectnet-camera/CMakeFiles/detectnet-camera.dir/rule

.PHONY : detectnet-camera

# fast build rule for target.
detectnet-camera/fast:
	cd /home/jaewoo/Desktop/project/jetson-inference && $(MAKE) -f detectnet-camera/CMakeFiles/detectnet-camera.dir/build.make detectnet-camera/CMakeFiles/detectnet-camera.dir/build
.PHONY : detectnet-camera/fast

detectnet-camera.o: detectnet-camera.cpp.o

.PHONY : detectnet-camera.o

# target to build an object file
detectnet-camera.cpp.o:
	cd /home/jaewoo/Desktop/project/jetson-inference && $(MAKE) -f detectnet-camera/CMakeFiles/detectnet-camera.dir/build.make detectnet-camera/CMakeFiles/detectnet-camera.dir/detectnet-camera.cpp.o
.PHONY : detectnet-camera.cpp.o

detectnet-camera.i: detectnet-camera.cpp.i

.PHONY : detectnet-camera.i

# target to preprocess a source file
detectnet-camera.cpp.i:
	cd /home/jaewoo/Desktop/project/jetson-inference && $(MAKE) -f detectnet-camera/CMakeFiles/detectnet-camera.dir/build.make detectnet-camera/CMakeFiles/detectnet-camera.dir/detectnet-camera.cpp.i
.PHONY : detectnet-camera.cpp.i

detectnet-camera.s: detectnet-camera.cpp.s

.PHONY : detectnet-camera.s

# target to generate assembly for a file
detectnet-camera.cpp.s:
	cd /home/jaewoo/Desktop/project/jetson-inference && $(MAKE) -f detectnet-camera/CMakeFiles/detectnet-camera.dir/build.make detectnet-camera/CMakeFiles/detectnet-camera.dir/detectnet-camera.cpp.s
.PHONY : detectnet-camera.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... install/strip"
	@echo "... edit_cache"
	@echo "... detectnet-camera"
	@echo "... rebuild_cache"
	@echo "... list_install_components"
	@echo "... install/local"
	@echo "... install"
	@echo "... detectnet-camera.o"
	@echo "... detectnet-camera.i"
	@echo "... detectnet-camera.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	cd /home/jaewoo/Desktop/project/jetson-inference && $(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
