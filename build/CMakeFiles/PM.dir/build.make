# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.21.3_1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.21.3_1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/yanruoyu/Documents/Graphics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/yanruoyu/Documents/Graphics/build

# Include any dependencies generated for this target.
include CMakeFiles/PM.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/PM.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/PM.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/PM.dir/flags.make

CMakeFiles/PM.dir/src/image.cpp.o: CMakeFiles/PM.dir/flags.make
CMakeFiles/PM.dir/src/image.cpp.o: ../src/image.cpp
CMakeFiles/PM.dir/src/image.cpp.o: CMakeFiles/PM.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/yanruoyu/Documents/Graphics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/PM.dir/src/image.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/PM.dir/src/image.cpp.o -MF CMakeFiles/PM.dir/src/image.cpp.o.d -o CMakeFiles/PM.dir/src/image.cpp.o -c /Users/yanruoyu/Documents/Graphics/src/image.cpp

CMakeFiles/PM.dir/src/image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PM.dir/src/image.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/yanruoyu/Documents/Graphics/src/image.cpp > CMakeFiles/PM.dir/src/image.cpp.i

CMakeFiles/PM.dir/src/image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PM.dir/src/image.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/yanruoyu/Documents/Graphics/src/image.cpp -o CMakeFiles/PM.dir/src/image.cpp.s

CMakeFiles/PM.dir/src/main.cpp.o: CMakeFiles/PM.dir/flags.make
CMakeFiles/PM.dir/src/main.cpp.o: ../src/main.cpp
CMakeFiles/PM.dir/src/main.cpp.o: CMakeFiles/PM.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/yanruoyu/Documents/Graphics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/PM.dir/src/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/PM.dir/src/main.cpp.o -MF CMakeFiles/PM.dir/src/main.cpp.o.d -o CMakeFiles/PM.dir/src/main.cpp.o -c /Users/yanruoyu/Documents/Graphics/src/main.cpp

CMakeFiles/PM.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PM.dir/src/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/yanruoyu/Documents/Graphics/src/main.cpp > CMakeFiles/PM.dir/src/main.cpp.i

CMakeFiles/PM.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PM.dir/src/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/yanruoyu/Documents/Graphics/src/main.cpp -o CMakeFiles/PM.dir/src/main.cpp.s

CMakeFiles/PM.dir/src/mesh.cpp.o: CMakeFiles/PM.dir/flags.make
CMakeFiles/PM.dir/src/mesh.cpp.o: ../src/mesh.cpp
CMakeFiles/PM.dir/src/mesh.cpp.o: CMakeFiles/PM.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/yanruoyu/Documents/Graphics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/PM.dir/src/mesh.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/PM.dir/src/mesh.cpp.o -MF CMakeFiles/PM.dir/src/mesh.cpp.o.d -o CMakeFiles/PM.dir/src/mesh.cpp.o -c /Users/yanruoyu/Documents/Graphics/src/mesh.cpp

CMakeFiles/PM.dir/src/mesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PM.dir/src/mesh.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/yanruoyu/Documents/Graphics/src/mesh.cpp > CMakeFiles/PM.dir/src/mesh.cpp.i

CMakeFiles/PM.dir/src/mesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PM.dir/src/mesh.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/yanruoyu/Documents/Graphics/src/mesh.cpp -o CMakeFiles/PM.dir/src/mesh.cpp.s

CMakeFiles/PM.dir/src/scene_parser.cpp.o: CMakeFiles/PM.dir/flags.make
CMakeFiles/PM.dir/src/scene_parser.cpp.o: ../src/scene_parser.cpp
CMakeFiles/PM.dir/src/scene_parser.cpp.o: CMakeFiles/PM.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/yanruoyu/Documents/Graphics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/PM.dir/src/scene_parser.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/PM.dir/src/scene_parser.cpp.o -MF CMakeFiles/PM.dir/src/scene_parser.cpp.o.d -o CMakeFiles/PM.dir/src/scene_parser.cpp.o -c /Users/yanruoyu/Documents/Graphics/src/scene_parser.cpp

CMakeFiles/PM.dir/src/scene_parser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PM.dir/src/scene_parser.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/yanruoyu/Documents/Graphics/src/scene_parser.cpp > CMakeFiles/PM.dir/src/scene_parser.cpp.i

CMakeFiles/PM.dir/src/scene_parser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PM.dir/src/scene_parser.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/yanruoyu/Documents/Graphics/src/scene_parser.cpp -o CMakeFiles/PM.dir/src/scene_parser.cpp.s

CMakeFiles/PM.dir/src/render.cpp.o: CMakeFiles/PM.dir/flags.make
CMakeFiles/PM.dir/src/render.cpp.o: ../src/render.cpp
CMakeFiles/PM.dir/src/render.cpp.o: CMakeFiles/PM.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/yanruoyu/Documents/Graphics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/PM.dir/src/render.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/PM.dir/src/render.cpp.o -MF CMakeFiles/PM.dir/src/render.cpp.o.d -o CMakeFiles/PM.dir/src/render.cpp.o -c /Users/yanruoyu/Documents/Graphics/src/render.cpp

CMakeFiles/PM.dir/src/render.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PM.dir/src/render.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/yanruoyu/Documents/Graphics/src/render.cpp > CMakeFiles/PM.dir/src/render.cpp.i

CMakeFiles/PM.dir/src/render.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PM.dir/src/render.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/yanruoyu/Documents/Graphics/src/render.cpp -o CMakeFiles/PM.dir/src/render.cpp.s

CMakeFiles/PM.dir/src/utils.cpp.o: CMakeFiles/PM.dir/flags.make
CMakeFiles/PM.dir/src/utils.cpp.o: ../src/utils.cpp
CMakeFiles/PM.dir/src/utils.cpp.o: CMakeFiles/PM.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/yanruoyu/Documents/Graphics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/PM.dir/src/utils.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/PM.dir/src/utils.cpp.o -MF CMakeFiles/PM.dir/src/utils.cpp.o.d -o CMakeFiles/PM.dir/src/utils.cpp.o -c /Users/yanruoyu/Documents/Graphics/src/utils.cpp

CMakeFiles/PM.dir/src/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PM.dir/src/utils.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/yanruoyu/Documents/Graphics/src/utils.cpp > CMakeFiles/PM.dir/src/utils.cpp.i

CMakeFiles/PM.dir/src/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PM.dir/src/utils.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/yanruoyu/Documents/Graphics/src/utils.cpp -o CMakeFiles/PM.dir/src/utils.cpp.s

# Object files for target PM
PM_OBJECTS = \
"CMakeFiles/PM.dir/src/image.cpp.o" \
"CMakeFiles/PM.dir/src/main.cpp.o" \
"CMakeFiles/PM.dir/src/mesh.cpp.o" \
"CMakeFiles/PM.dir/src/scene_parser.cpp.o" \
"CMakeFiles/PM.dir/src/render.cpp.o" \
"CMakeFiles/PM.dir/src/utils.cpp.o"

# External object files for target PM
PM_EXTERNAL_OBJECTS =

../bin/PM: CMakeFiles/PM.dir/src/image.cpp.o
../bin/PM: CMakeFiles/PM.dir/src/main.cpp.o
../bin/PM: CMakeFiles/PM.dir/src/mesh.cpp.o
../bin/PM: CMakeFiles/PM.dir/src/scene_parser.cpp.o
../bin/PM: CMakeFiles/PM.dir/src/render.cpp.o
../bin/PM: CMakeFiles/PM.dir/src/utils.cpp.o
../bin/PM: CMakeFiles/PM.dir/build.make
../bin/PM: deps/vecmath/libvecmath.a
../bin/PM: CMakeFiles/PM.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/yanruoyu/Documents/Graphics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable ../bin/PM"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PM.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/PM.dir/build: ../bin/PM
.PHONY : CMakeFiles/PM.dir/build

CMakeFiles/PM.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PM.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PM.dir/clean

CMakeFiles/PM.dir/depend:
	cd /Users/yanruoyu/Documents/Graphics/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/yanruoyu/Documents/Graphics /Users/yanruoyu/Documents/Graphics /Users/yanruoyu/Documents/Graphics/build /Users/yanruoyu/Documents/Graphics/build /Users/yanruoyu/Documents/Graphics/build/CMakeFiles/PM.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PM.dir/depend

