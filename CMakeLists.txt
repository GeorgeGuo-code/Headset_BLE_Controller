# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# The gesture/sensor/console components live one level deeper than the default
# `components/` search path (in `components/components/`), so register that dir.
set(EXTRA_COMPONENT_DIRS "components/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sample_project)
