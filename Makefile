# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /playpen/wilkie/StochGraph

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /playpen/wilkie/StochGraph

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running interactive CMake command-line interface..."
	/usr/bin/cmake -i .
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

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

# Special rule for the target install/local
install/local: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local

# Special rule for the target install/local
install/local/fast: install/local
.PHONY : install/local/fast

# Special rule for the target install/strip
install/strip: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip

# Special rule for the target install/strip
install/strip/fast: install/strip
.PHONY : install/strip/fast

# Special rule for the target list_install_components
list_install_components:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Available install components are: \"Unspecified\""
.PHONY : list_install_components

# Special rule for the target list_install_components
list_install_components/fast: list_install_components
.PHONY : list_install_components/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# Special rule for the target test
test:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running tests..."
	/usr/bin/ctest --force-new-ctest-process $(ARGS)
.PHONY : test

# Special rule for the target test
test/fast: test
.PHONY : test/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /playpen/wilkie/StochGraph/CMakeFiles /playpen/wilkie/StochGraph/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /playpen/wilkie/StochGraph/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named Continuous

# Build rule for target.
Continuous: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 Continuous
.PHONY : Continuous

# fast build rule for target.
Continuous/fast:
	$(MAKE) -f CMakeFiles/Continuous.dir/build.make CMakeFiles/Continuous.dir/build
.PHONY : Continuous/fast

#=============================================================================
# Target rules for targets named ContinuousBuild

# Build rule for target.
ContinuousBuild: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ContinuousBuild
.PHONY : ContinuousBuild

# fast build rule for target.
ContinuousBuild/fast:
	$(MAKE) -f CMakeFiles/ContinuousBuild.dir/build.make CMakeFiles/ContinuousBuild.dir/build
.PHONY : ContinuousBuild/fast

#=============================================================================
# Target rules for targets named ContinuousConfigure

# Build rule for target.
ContinuousConfigure: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ContinuousConfigure
.PHONY : ContinuousConfigure

# fast build rule for target.
ContinuousConfigure/fast:
	$(MAKE) -f CMakeFiles/ContinuousConfigure.dir/build.make CMakeFiles/ContinuousConfigure.dir/build
.PHONY : ContinuousConfigure/fast

#=============================================================================
# Target rules for targets named ContinuousCoverage

# Build rule for target.
ContinuousCoverage: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ContinuousCoverage
.PHONY : ContinuousCoverage

# fast build rule for target.
ContinuousCoverage/fast:
	$(MAKE) -f CMakeFiles/ContinuousCoverage.dir/build.make CMakeFiles/ContinuousCoverage.dir/build
.PHONY : ContinuousCoverage/fast

#=============================================================================
# Target rules for targets named ContinuousMemCheck

# Build rule for target.
ContinuousMemCheck: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ContinuousMemCheck
.PHONY : ContinuousMemCheck

# fast build rule for target.
ContinuousMemCheck/fast:
	$(MAKE) -f CMakeFiles/ContinuousMemCheck.dir/build.make CMakeFiles/ContinuousMemCheck.dir/build
.PHONY : ContinuousMemCheck/fast

#=============================================================================
# Target rules for targets named ContinuousStart

# Build rule for target.
ContinuousStart: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ContinuousStart
.PHONY : ContinuousStart

# fast build rule for target.
ContinuousStart/fast:
	$(MAKE) -f CMakeFiles/ContinuousStart.dir/build.make CMakeFiles/ContinuousStart.dir/build
.PHONY : ContinuousStart/fast

#=============================================================================
# Target rules for targets named ContinuousSubmit

# Build rule for target.
ContinuousSubmit: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ContinuousSubmit
.PHONY : ContinuousSubmit

# fast build rule for target.
ContinuousSubmit/fast:
	$(MAKE) -f CMakeFiles/ContinuousSubmit.dir/build.make CMakeFiles/ContinuousSubmit.dir/build
.PHONY : ContinuousSubmit/fast

#=============================================================================
# Target rules for targets named ContinuousTest

# Build rule for target.
ContinuousTest: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ContinuousTest
.PHONY : ContinuousTest

# fast build rule for target.
ContinuousTest/fast:
	$(MAKE) -f CMakeFiles/ContinuousTest.dir/build.make CMakeFiles/ContinuousTest.dir/build
.PHONY : ContinuousTest/fast

#=============================================================================
# Target rules for targets named ContinuousUpdate

# Build rule for target.
ContinuousUpdate: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ContinuousUpdate
.PHONY : ContinuousUpdate

# fast build rule for target.
ContinuousUpdate/fast:
	$(MAKE) -f CMakeFiles/ContinuousUpdate.dir/build.make CMakeFiles/ContinuousUpdate.dir/build
.PHONY : ContinuousUpdate/fast

#=============================================================================
# Target rules for targets named Experimental

# Build rule for target.
Experimental: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 Experimental
.PHONY : Experimental

# fast build rule for target.
Experimental/fast:
	$(MAKE) -f CMakeFiles/Experimental.dir/build.make CMakeFiles/Experimental.dir/build
.PHONY : Experimental/fast

#=============================================================================
# Target rules for targets named ExperimentalBuild

# Build rule for target.
ExperimentalBuild: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ExperimentalBuild
.PHONY : ExperimentalBuild

# fast build rule for target.
ExperimentalBuild/fast:
	$(MAKE) -f CMakeFiles/ExperimentalBuild.dir/build.make CMakeFiles/ExperimentalBuild.dir/build
.PHONY : ExperimentalBuild/fast

#=============================================================================
# Target rules for targets named ExperimentalConfigure

# Build rule for target.
ExperimentalConfigure: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ExperimentalConfigure
.PHONY : ExperimentalConfigure

# fast build rule for target.
ExperimentalConfigure/fast:
	$(MAKE) -f CMakeFiles/ExperimentalConfigure.dir/build.make CMakeFiles/ExperimentalConfigure.dir/build
.PHONY : ExperimentalConfigure/fast

#=============================================================================
# Target rules for targets named ExperimentalCoverage

# Build rule for target.
ExperimentalCoverage: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ExperimentalCoverage
.PHONY : ExperimentalCoverage

# fast build rule for target.
ExperimentalCoverage/fast:
	$(MAKE) -f CMakeFiles/ExperimentalCoverage.dir/build.make CMakeFiles/ExperimentalCoverage.dir/build
.PHONY : ExperimentalCoverage/fast

#=============================================================================
# Target rules for targets named ExperimentalMemCheck

# Build rule for target.
ExperimentalMemCheck: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ExperimentalMemCheck
.PHONY : ExperimentalMemCheck

# fast build rule for target.
ExperimentalMemCheck/fast:
	$(MAKE) -f CMakeFiles/ExperimentalMemCheck.dir/build.make CMakeFiles/ExperimentalMemCheck.dir/build
.PHONY : ExperimentalMemCheck/fast

#=============================================================================
# Target rules for targets named ExperimentalStart

# Build rule for target.
ExperimentalStart: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ExperimentalStart
.PHONY : ExperimentalStart

# fast build rule for target.
ExperimentalStart/fast:
	$(MAKE) -f CMakeFiles/ExperimentalStart.dir/build.make CMakeFiles/ExperimentalStart.dir/build
.PHONY : ExperimentalStart/fast

#=============================================================================
# Target rules for targets named ExperimentalSubmit

# Build rule for target.
ExperimentalSubmit: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ExperimentalSubmit
.PHONY : ExperimentalSubmit

# fast build rule for target.
ExperimentalSubmit/fast:
	$(MAKE) -f CMakeFiles/ExperimentalSubmit.dir/build.make CMakeFiles/ExperimentalSubmit.dir/build
.PHONY : ExperimentalSubmit/fast

#=============================================================================
# Target rules for targets named ExperimentalTest

# Build rule for target.
ExperimentalTest: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ExperimentalTest
.PHONY : ExperimentalTest

# fast build rule for target.
ExperimentalTest/fast:
	$(MAKE) -f CMakeFiles/ExperimentalTest.dir/build.make CMakeFiles/ExperimentalTest.dir/build
.PHONY : ExperimentalTest/fast

#=============================================================================
# Target rules for targets named ExperimentalUpdate

# Build rule for target.
ExperimentalUpdate: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ExperimentalUpdate
.PHONY : ExperimentalUpdate

# fast build rule for target.
ExperimentalUpdate/fast:
	$(MAKE) -f CMakeFiles/ExperimentalUpdate.dir/build.make CMakeFiles/ExperimentalUpdate.dir/build
.PHONY : ExperimentalUpdate/fast

#=============================================================================
# Target rules for targets named LocalFrameUnitTests

# Build rule for target.
LocalFrameUnitTests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 LocalFrameUnitTests
.PHONY : LocalFrameUnitTests

# fast build rule for target.
LocalFrameUnitTests/fast:
	$(MAKE) -f CMakeFiles/LocalFrameUnitTests.dir/build.make CMakeFiles/LocalFrameUnitTests.dir/build
.PHONY : LocalFrameUnitTests/fast

#=============================================================================
# Target rules for targets named Nightly

# Build rule for target.
Nightly: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 Nightly
.PHONY : Nightly

# fast build rule for target.
Nightly/fast:
	$(MAKE) -f CMakeFiles/Nightly.dir/build.make CMakeFiles/Nightly.dir/build
.PHONY : Nightly/fast

#=============================================================================
# Target rules for targets named NightlyBuild

# Build rule for target.
NightlyBuild: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 NightlyBuild
.PHONY : NightlyBuild

# fast build rule for target.
NightlyBuild/fast:
	$(MAKE) -f CMakeFiles/NightlyBuild.dir/build.make CMakeFiles/NightlyBuild.dir/build
.PHONY : NightlyBuild/fast

#=============================================================================
# Target rules for targets named NightlyConfigure

# Build rule for target.
NightlyConfigure: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 NightlyConfigure
.PHONY : NightlyConfigure

# fast build rule for target.
NightlyConfigure/fast:
	$(MAKE) -f CMakeFiles/NightlyConfigure.dir/build.make CMakeFiles/NightlyConfigure.dir/build
.PHONY : NightlyConfigure/fast

#=============================================================================
# Target rules for targets named NightlyCoverage

# Build rule for target.
NightlyCoverage: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 NightlyCoverage
.PHONY : NightlyCoverage

# fast build rule for target.
NightlyCoverage/fast:
	$(MAKE) -f CMakeFiles/NightlyCoverage.dir/build.make CMakeFiles/NightlyCoverage.dir/build
.PHONY : NightlyCoverage/fast

#=============================================================================
# Target rules for targets named NightlyMemCheck

# Build rule for target.
NightlyMemCheck: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 NightlyMemCheck
.PHONY : NightlyMemCheck

# fast build rule for target.
NightlyMemCheck/fast:
	$(MAKE) -f CMakeFiles/NightlyMemCheck.dir/build.make CMakeFiles/NightlyMemCheck.dir/build
.PHONY : NightlyMemCheck/fast

#=============================================================================
# Target rules for targets named NightlyMemoryCheck

# Build rule for target.
NightlyMemoryCheck: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 NightlyMemoryCheck
.PHONY : NightlyMemoryCheck

# fast build rule for target.
NightlyMemoryCheck/fast:
	$(MAKE) -f CMakeFiles/NightlyMemoryCheck.dir/build.make CMakeFiles/NightlyMemoryCheck.dir/build
.PHONY : NightlyMemoryCheck/fast

#=============================================================================
# Target rules for targets named NightlyStart

# Build rule for target.
NightlyStart: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 NightlyStart
.PHONY : NightlyStart

# fast build rule for target.
NightlyStart/fast:
	$(MAKE) -f CMakeFiles/NightlyStart.dir/build.make CMakeFiles/NightlyStart.dir/build
.PHONY : NightlyStart/fast

#=============================================================================
# Target rules for targets named NightlySubmit

# Build rule for target.
NightlySubmit: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 NightlySubmit
.PHONY : NightlySubmit

# fast build rule for target.
NightlySubmit/fast:
	$(MAKE) -f CMakeFiles/NightlySubmit.dir/build.make CMakeFiles/NightlySubmit.dir/build
.PHONY : NightlySubmit/fast

#=============================================================================
# Target rules for targets named NightlyTest

# Build rule for target.
NightlyTest: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 NightlyTest
.PHONY : NightlyTest

# fast build rule for target.
NightlyTest/fast:
	$(MAKE) -f CMakeFiles/NightlyTest.dir/build.make CMakeFiles/NightlyTest.dir/build
.PHONY : NightlyTest/fast

#=============================================================================
# Target rules for targets named NightlyUpdate

# Build rule for target.
NightlyUpdate: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 NightlyUpdate
.PHONY : NightlyUpdate

# fast build rule for target.
NightlyUpdate/fast:
	$(MAKE) -f CMakeFiles/NightlyUpdate.dir/build.make CMakeFiles/NightlyUpdate.dir/build
.PHONY : NightlyUpdate/fast

#=============================================================================
# Target rules for targets named StochEdgeUnitTests

# Build rule for target.
StochEdgeUnitTests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 StochEdgeUnitTests
.PHONY : StochEdgeUnitTests

# fast build rule for target.
StochEdgeUnitTests/fast:
	$(MAKE) -f CMakeFiles/StochEdgeUnitTests.dir/build.make CMakeFiles/StochEdgeUnitTests.dir/build
.PHONY : StochEdgeUnitTests/fast

#=============================================================================
# Target rules for targets named StochGraphUnitTests

# Build rule for target.
StochGraphUnitTests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 StochGraphUnitTests
.PHONY : StochGraphUnitTests

# fast build rule for target.
StochGraphUnitTests/fast:
	$(MAKE) -f CMakeFiles/StochGraphUnitTests.dir/build.make CMakeFiles/StochGraphUnitTests.dir/build
.PHONY : StochGraphUnitTests/fast

#=============================================================================
# Target rules for targets named SumoUtilUnitTests

# Build rule for target.
SumoUtilUnitTests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 SumoUtilUnitTests
.PHONY : SumoUtilUnitTests

# fast build rule for target.
SumoUtilUnitTests/fast:
	$(MAKE) -f CMakeFiles/SumoUtilUnitTests.dir/build.make CMakeFiles/SumoUtilUnitTests.dir/build
.PHONY : SumoUtilUnitTests/fast

#=============================================================================
# Target rules for targets named stoch_graph

# Build rule for target.
stoch_graph: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 stoch_graph
.PHONY : stoch_graph

# fast build rule for target.
stoch_graph/fast:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/build
.PHONY : stoch_graph/fast

local_frame.o: local_frame.cpp.o
.PHONY : local_frame.o

# target to build an object file
local_frame.cpp.o:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/local_frame.cpp.o
.PHONY : local_frame.cpp.o

local_frame.i: local_frame.cpp.i
.PHONY : local_frame.i

# target to preprocess a source file
local_frame.cpp.i:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/local_frame.cpp.i
.PHONY : local_frame.cpp.i

local_frame.s: local_frame.cpp.s
.PHONY : local_frame.s

# target to generate assembly for a file
local_frame.cpp.s:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/local_frame.cpp.s
.PHONY : local_frame.cpp.s

local_frame_unit_tests.o: local_frame_unit_tests.cpp.o
.PHONY : local_frame_unit_tests.o

# target to build an object file
local_frame_unit_tests.cpp.o:
	$(MAKE) -f CMakeFiles/LocalFrameUnitTests.dir/build.make CMakeFiles/LocalFrameUnitTests.dir/local_frame_unit_tests.cpp.o
.PHONY : local_frame_unit_tests.cpp.o

local_frame_unit_tests.i: local_frame_unit_tests.cpp.i
.PHONY : local_frame_unit_tests.i

# target to preprocess a source file
local_frame_unit_tests.cpp.i:
	$(MAKE) -f CMakeFiles/LocalFrameUnitTests.dir/build.make CMakeFiles/LocalFrameUnitTests.dir/local_frame_unit_tests.cpp.i
.PHONY : local_frame_unit_tests.cpp.i

local_frame_unit_tests.s: local_frame_unit_tests.cpp.s
.PHONY : local_frame_unit_tests.s

# target to generate assembly for a file
local_frame_unit_tests.cpp.s:
	$(MAKE) -f CMakeFiles/LocalFrameUnitTests.dir/build.make CMakeFiles/LocalFrameUnitTests.dir/local_frame_unit_tests.cpp.s
.PHONY : local_frame_unit_tests.cpp.s

stoch_edge.o: stoch_edge.cpp.o
.PHONY : stoch_edge.o

# target to build an object file
stoch_edge.cpp.o:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/stoch_edge.cpp.o
.PHONY : stoch_edge.cpp.o

stoch_edge.i: stoch_edge.cpp.i
.PHONY : stoch_edge.i

# target to preprocess a source file
stoch_edge.cpp.i:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/stoch_edge.cpp.i
.PHONY : stoch_edge.cpp.i

stoch_edge.s: stoch_edge.cpp.s
.PHONY : stoch_edge.s

# target to generate assembly for a file
stoch_edge.cpp.s:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/stoch_edge.cpp.s
.PHONY : stoch_edge.cpp.s

stoch_edge_unit_tests.o: stoch_edge_unit_tests.cpp.o
.PHONY : stoch_edge_unit_tests.o

# target to build an object file
stoch_edge_unit_tests.cpp.o:
	$(MAKE) -f CMakeFiles/StochEdgeUnitTests.dir/build.make CMakeFiles/StochEdgeUnitTests.dir/stoch_edge_unit_tests.cpp.o
.PHONY : stoch_edge_unit_tests.cpp.o

stoch_edge_unit_tests.i: stoch_edge_unit_tests.cpp.i
.PHONY : stoch_edge_unit_tests.i

# target to preprocess a source file
stoch_edge_unit_tests.cpp.i:
	$(MAKE) -f CMakeFiles/StochEdgeUnitTests.dir/build.make CMakeFiles/StochEdgeUnitTests.dir/stoch_edge_unit_tests.cpp.i
.PHONY : stoch_edge_unit_tests.cpp.i

stoch_edge_unit_tests.s: stoch_edge_unit_tests.cpp.s
.PHONY : stoch_edge_unit_tests.s

# target to generate assembly for a file
stoch_edge_unit_tests.cpp.s:
	$(MAKE) -f CMakeFiles/StochEdgeUnitTests.dir/build.make CMakeFiles/StochEdgeUnitTests.dir/stoch_edge_unit_tests.cpp.s
.PHONY : stoch_edge_unit_tests.cpp.s

stoch_graph.o: stoch_graph.cpp.o
.PHONY : stoch_graph.o

# target to build an object file
stoch_graph.cpp.o:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/stoch_graph.cpp.o
.PHONY : stoch_graph.cpp.o

stoch_graph.i: stoch_graph.cpp.i
.PHONY : stoch_graph.i

# target to preprocess a source file
stoch_graph.cpp.i:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/stoch_graph.cpp.i
.PHONY : stoch_graph.cpp.i

stoch_graph.s: stoch_graph.cpp.s
.PHONY : stoch_graph.s

# target to generate assembly for a file
stoch_graph.cpp.s:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/stoch_graph.cpp.s
.PHONY : stoch_graph.cpp.s

stoch_graph_unit_tests.o: stoch_graph_unit_tests.cpp.o
.PHONY : stoch_graph_unit_tests.o

# target to build an object file
stoch_graph_unit_tests.cpp.o:
	$(MAKE) -f CMakeFiles/StochGraphUnitTests.dir/build.make CMakeFiles/StochGraphUnitTests.dir/stoch_graph_unit_tests.cpp.o
.PHONY : stoch_graph_unit_tests.cpp.o

stoch_graph_unit_tests.i: stoch_graph_unit_tests.cpp.i
.PHONY : stoch_graph_unit_tests.i

# target to preprocess a source file
stoch_graph_unit_tests.cpp.i:
	$(MAKE) -f CMakeFiles/StochGraphUnitTests.dir/build.make CMakeFiles/StochGraphUnitTests.dir/stoch_graph_unit_tests.cpp.i
.PHONY : stoch_graph_unit_tests.cpp.i

stoch_graph_unit_tests.s: stoch_graph_unit_tests.cpp.s
.PHONY : stoch_graph_unit_tests.s

# target to generate assembly for a file
stoch_graph_unit_tests.cpp.s:
	$(MAKE) -f CMakeFiles/StochGraphUnitTests.dir/build.make CMakeFiles/StochGraphUnitTests.dir/stoch_graph_unit_tests.cpp.s
.PHONY : stoch_graph_unit_tests.cpp.s

sumo_util.o: sumo_util.cpp.o
.PHONY : sumo_util.o

# target to build an object file
sumo_util.cpp.o:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/sumo_util.cpp.o
.PHONY : sumo_util.cpp.o

sumo_util.i: sumo_util.cpp.i
.PHONY : sumo_util.i

# target to preprocess a source file
sumo_util.cpp.i:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/sumo_util.cpp.i
.PHONY : sumo_util.cpp.i

sumo_util.s: sumo_util.cpp.s
.PHONY : sumo_util.s

# target to generate assembly for a file
sumo_util.cpp.s:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/sumo_util.cpp.s
.PHONY : sumo_util.cpp.s

sumo_util_unit_tests.o: sumo_util_unit_tests.cpp.o
.PHONY : sumo_util_unit_tests.o

# target to build an object file
sumo_util_unit_tests.cpp.o:
	$(MAKE) -f CMakeFiles/SumoUtilUnitTests.dir/build.make CMakeFiles/SumoUtilUnitTests.dir/sumo_util_unit_tests.cpp.o
.PHONY : sumo_util_unit_tests.cpp.o

sumo_util_unit_tests.i: sumo_util_unit_tests.cpp.i
.PHONY : sumo_util_unit_tests.i

# target to preprocess a source file
sumo_util_unit_tests.cpp.i:
	$(MAKE) -f CMakeFiles/SumoUtilUnitTests.dir/build.make CMakeFiles/SumoUtilUnitTests.dir/sumo_util_unit_tests.cpp.i
.PHONY : sumo_util_unit_tests.cpp.i

sumo_util_unit_tests.s: sumo_util_unit_tests.cpp.s
.PHONY : sumo_util_unit_tests.s

# target to generate assembly for a file
sumo_util_unit_tests.cpp.s:
	$(MAKE) -f CMakeFiles/SumoUtilUnitTests.dir/build.make CMakeFiles/SumoUtilUnitTests.dir/sumo_util_unit_tests.cpp.s
.PHONY : sumo_util_unit_tests.cpp.s

velocity_map.o: velocity_map.cpp.o
.PHONY : velocity_map.o

# target to build an object file
velocity_map.cpp.o:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/velocity_map.cpp.o
.PHONY : velocity_map.cpp.o

velocity_map.i: velocity_map.cpp.i
.PHONY : velocity_map.i

# target to preprocess a source file
velocity_map.cpp.i:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/velocity_map.cpp.i
.PHONY : velocity_map.cpp.i

velocity_map.s: velocity_map.cpp.s
.PHONY : velocity_map.s

# target to generate assembly for a file
velocity_map.cpp.s:
	$(MAKE) -f CMakeFiles/stoch_graph.dir/build.make CMakeFiles/stoch_graph.dir/velocity_map.cpp.s
.PHONY : velocity_map.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... Continuous"
	@echo "... ContinuousBuild"
	@echo "... ContinuousConfigure"
	@echo "... ContinuousCoverage"
	@echo "... ContinuousMemCheck"
	@echo "... ContinuousStart"
	@echo "... ContinuousSubmit"
	@echo "... ContinuousTest"
	@echo "... ContinuousUpdate"
	@echo "... Experimental"
	@echo "... ExperimentalBuild"
	@echo "... ExperimentalConfigure"
	@echo "... ExperimentalCoverage"
	@echo "... ExperimentalMemCheck"
	@echo "... ExperimentalStart"
	@echo "... ExperimentalSubmit"
	@echo "... ExperimentalTest"
	@echo "... ExperimentalUpdate"
	@echo "... LocalFrameUnitTests"
	@echo "... Nightly"
	@echo "... NightlyBuild"
	@echo "... NightlyConfigure"
	@echo "... NightlyCoverage"
	@echo "... NightlyMemCheck"
	@echo "... NightlyMemoryCheck"
	@echo "... NightlyStart"
	@echo "... NightlySubmit"
	@echo "... NightlyTest"
	@echo "... NightlyUpdate"
	@echo "... StochEdgeUnitTests"
	@echo "... StochGraphUnitTests"
	@echo "... SumoUtilUnitTests"
	@echo "... edit_cache"
	@echo "... install"
	@echo "... install/local"
	@echo "... install/strip"
	@echo "... list_install_components"
	@echo "... rebuild_cache"
	@echo "... stoch_graph"
	@echo "... test"
	@echo "... local_frame.o"
	@echo "... local_frame.i"
	@echo "... local_frame.s"
	@echo "... local_frame_unit_tests.o"
	@echo "... local_frame_unit_tests.i"
	@echo "... local_frame_unit_tests.s"
	@echo "... stoch_edge.o"
	@echo "... stoch_edge.i"
	@echo "... stoch_edge.s"
	@echo "... stoch_edge_unit_tests.o"
	@echo "... stoch_edge_unit_tests.i"
	@echo "... stoch_edge_unit_tests.s"
	@echo "... stoch_graph.o"
	@echo "... stoch_graph.i"
	@echo "... stoch_graph.s"
	@echo "... stoch_graph_unit_tests.o"
	@echo "... stoch_graph_unit_tests.i"
	@echo "... stoch_graph_unit_tests.s"
	@echo "... sumo_util.o"
	@echo "... sumo_util.i"
	@echo "... sumo_util.s"
	@echo "... sumo_util_unit_tests.o"
	@echo "... sumo_util_unit_tests.i"
	@echo "... sumo_util_unit_tests.s"
	@echo "... velocity_map.o"
	@echo "... velocity_map.i"
	@echo "... velocity_map.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

