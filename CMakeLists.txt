
cmake_minimum_required(VERSION 3.21)

project(BalloonInterceptor
    VERSION 0.4
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

qt_standard_project_setup()

# ============================================================
# Main Qt application
# ============================================================

qt_add_executable(BalloonInterceptor
    src/main.cpp
    src/ui/SimulationWidget.h
    src/ui/SimulationWidget.cpp
    src/ui/TelemetryWidget.h
    src/ui/TelemetryWidget.cpp
    src/physics/Rocket.h
    src/physics/Rocket.cpp
    src/physics/Balloon.h
    src/physics/Balloon.cpp
    src/simulator/Simulator.h
    src/simulator/Simulator.cpp
    src/controller/Controller.h
    src/controller/SimpleController.h
    src/controller/SimpleController.cpp
    src/controller/RL_Controller.h
    src/controller/RL_Controller.cpp
    src/physics/MovingObject.h
    src/physics/MovingObject.cpp
    src/config.h
)

target_link_libraries(BalloonInterceptor PRIVATE
    Qt6::Widgets
)

set_target_properties(BalloonInterceptor PROPERTIES
    MACOSX_BUNDLE TRUE
)

# ============================================================
# ONNX Runtime
# ============================================================

execute_process(
    COMMAND brew --prefix onnxruntime
    OUTPUT_VARIABLE ONNXRUNTIME_ROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE ONNXRUNTIME_BREW_RESULT
)

if(NOT ONNXRUNTIME_BREW_RESULT EQUAL 0)
    message(FATAL_ERROR
        "ONNX Runtime was not found through Homebrew. "
        "Install it with: brew install onnxruntime"
    )
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS
        "${ONNXRUNTIME_ROOT}/include"
        "${ONNXRUNTIME_ROOT}/include/onnxruntime/core/session"
    PATH_SUFFIXES
        onnxruntime
        onnxruntime/core/session
    REQUIRED
)

find_library(ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    PATHS "${ONNXRUNTIME_ROOT}/lib"
    REQUIRED
)

message(STATUS "ONNX Runtime root: ${ONNXRUNTIME_ROOT}")
message(STATUS "ONNX Runtime headers: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNX Runtime library: ${ONNXRUNTIME_LIBRARY}")

target_include_directories(BalloonInterceptor PRIVATE
    "${ONNXRUNTIME_INCLUDE_DIR}"
)

target_link_libraries(BalloonInterceptor PRIVATE
    "${ONNXRUNTIME_LIBRARY}"
)

# Help macOS locate ONNX Runtime at execution time.
set_target_properties(BalloonInterceptor PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
)

# ============================================================
# DQN ONNX model
# ============================================================

set(DQN_ONNX_MODEL
    "${CMAKE_CURRENT_SOURCE_DIR}/models/dqn_rocket_actor.onnx"
)

if(EXISTS "${DQN_ONNX_MODEL}")

    message(STATUS "DQN ONNX model found: ${DQN_ONNX_MODEL}")

    add_custom_command(TARGET BalloonInterceptor POST_BUILD

        # Create the destination directory if necessary.
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:BalloonInterceptor>"

        # Copy the trained DQN model.
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${DQN_ONNX_MODEL}"
            "$<TARGET_FILE_DIR:BalloonInterceptor>/dqn_rocket_actor.onnx"

        COMMENT "Copying DQN ONNX model beside the executable"
        VERBATIM
    )

else()

    message(WARNING
        "DQN ONNX model not found: ${DQN_ONNX_MODEL}\n"
        "Building without the ONNX model.\n"
        "After training, run python/export_model.py, "
        "then reconfigure and rebuild."
    )

endif()

# ============================================================
# Python extension for reinforcement learning
# ============================================================

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(rocket_sim
    src/python_bindings.cpp
    src/simulator/Simulator.cpp
    src/physics/Rocket.cpp
    src/physics/Balloon.cpp
    src/physics/MovingObject.cpp
)

target_include_directories(rocket_sim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${ONNXRUNTIME_INCLUDE_DIR}"
)

target_link_libraries(rocket_sim PRIVATE
    Qt6::Widgets
    "${ONNXRUNTIME_LIBRARY}"
)

# ============================================================
# Build configuration
# ============================================================

# Configure:
#
# cmake -S . -B build-rl \
#   -DCMAKE_PREFIX_PATH="$HOME/Qt/6.11.2/macos" \
#   -Dpybind11_DIR="$(python -m pybind11 --cmakedir)" \
#   -DPython_EXECUTABLE="$(which python)"
#
# Build Qt application:
#
# cmake --build build-rl --target BalloonInterceptor -j 4
#
# Build Python extension:
#
# cmake --build build-rl --target rocket_sim -j 4
