cmake_minimum_required(VERSION 3.13)
project(ltmu_jetson LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(EXE_NAME LtmuTracker)

file(GLOB_RECURSE SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_executable(${EXE_NAME} ${SOURCES})
set_target_properties(${EXE_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ── econ-cameras branch: real V4L2 ioctls (SET_CAMERA_PARAM) and the
#    Jetson hardware H.264 encoder instead of the uav-dataset branch's
#    software x264enc default. ─────────────────────────────────────────
target_compile_definitions(${EXE_NAME} PRIVATE
  LTMU_ECON_CAMERAS
  LTMU_ENCODER_ELEMENT="nvv4l2h264enc maxperf-enable=1 insert-sps-pps=1 iframeinterval=30 idrinterval=30 bitrate=8000000"
)

target_include_directories(${EXE_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ── OpenCV (must include the opencv_contrib tracking module for CSRT) ──
find_package(OpenCV REQUIRED)
target_link_libraries(${EXE_NAME} PRIVATE ${OpenCV_LIBS})
target_include_directories(${EXE_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

# ── GStreamer ────────────────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
target_include_directories(${EXE_NAME} PRIVATE ${GST_INCLUDE_DIRS})
target_compile_options(${EXE_NAME} PRIVATE ${GST_CFLAGS_OTHER})
target_link_libraries(${EXE_NAME} PRIVATE ${GST_LIBRARIES})

# ── ONNX Runtime ─────────────────────────────────────────────────────
# Jetson has no apt package for the C++ SDK. Point ONNXRUNTIME_ROOT at an
# extracted onnxruntime-linux-aarch64-<ver> (or -gpu) release tree, e.g.:
#   cmake -B build -DONNXRUNTIME_ROOT=/opt/onnxruntime-linux-aarch64-1.17.1
# See README.md "ONNX Runtime on Orin NX" for the download/build steps.
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to extracted ONNX Runtime release")
if(NOT ONNXRUNTIME_ROOT)
  message(FATAL_ERROR
    "ONNXRUNTIME_ROOT is not set. Pass -DONNXRUNTIME_ROOT=/path/to/onnxruntime "
    "(an extracted onnxruntime-linux-aarch64 release with include/ and lib/). "
    "See README.md.")
endif()
target_include_directories(${EXE_NAME} PRIVATE "${ONNXRUNTIME_ROOT}/include")
find_library(ONNXRUNTIME_LIB onnxruntime PATHS "${ONNXRUNTIME_ROOT}/lib" REQUIRED)
target_link_libraries(${EXE_NAME} PRIVATE ${ONNXRUNTIME_LIB})
target_link_directories(${EXE_NAME} PRIVATE "${ONNXRUNTIME_ROOT}/lib")

# ── pthread ──────────────────────────────────────────────────────────
find_package(Threads REQUIRED)
target_link_libraries(${EXE_NAME} PRIVATE Threads::Threads)

# ── std::filesystem (only needed on GCC < 9; JetPack 5's GCC folds it
#    into libstdc++ already, so only link it if the lib actually exists) ─
find_library(STDCPP_FS_LIB stdc++fs)
if(STDCPP_FS_LIB)
  target_link_libraries(${EXE_NAME} PRIVATE ${STDCPP_FS_LIB})
endif()

message(STATUS "LTMU tracker configured — OpenCV ${OpenCV_VERSION}, ONNXRUNTIME_ROOT=${ONNXRUNTIME_ROOT}")
