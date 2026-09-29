include_guard(GLOBAL)
include(ExternalProject)

set(VK_OPENCV_VERSION 4.14.0)
set(VK_OPENCV_SHA256 ee8fb9b30eb60850431b4656447080e3737b56e45719c92b67f245950609f86e)
set(VK_OPENCV_MODULES core imgproc calib3d features2d flann objdetect)
set(VK_OPENCV_EXCLUDED dnn highgui videoio imgcodecs)

if(NOT VISIONKIT_OPENCV_DIR AND DEFINED ENV{VISIONKIT_OPENCV_DIR})
    set(VISIONKIT_OPENCV_DIR "$ENV{VISIONKIT_OPENCV_DIR}")
endif()

if(VISIONKIT_OPENCV_DIR)
    set(_vk_opencv_prefix "${VISIONKIT_OPENCV_DIR}")
    if(NOT EXISTS "${_vk_opencv_prefix}/include/opencv4/opencv2/core.hpp")
        message(FATAL_ERROR "VISIONKIT_OPENCV_DIR must be an installed OpenCV prefix")
    endif()
else()
    set(_vk_opencv_prefix "/home/joao/dev/materia-deps/opencv/4.14.0-trimmed-v2")
    if(EXISTS "${_vk_opencv_prefix}/include/opencv4/opencv2/core.hpp"
        AND EXISTS "${_vk_opencv_prefix}/lib/libopencv_objdetect.a")
        set(VISIONKIT_OPENCV_DIR "${_vk_opencv_prefix}")
    endif()
endif()

if(NOT VISIONKIT_OPENCV_DIR)
    set(_vk_opencv_cmake_args
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DBUILD_SHARED_LIBS=OFF
        -DBUILD_LIST=core,imgproc,calib3d,features2d,flann,objdetect
        -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF
        -DBUILD_DOCS=OFF -DBUILD_opencv_apps=OFF
        -DBUILD_JAVA=OFF -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF
        -DWITH_IPP=OFF -DWITH_ADE=OFF -DWITH_ITT=OFF
        -DWITH_OPENCL=OFF -DWITH_CUDA=OFF -DWITH_TBB=OFF -DWITH_OPENMP=OFF
        -DWITH_GTK=OFF -DWITH_QT=OFF -DWITH_VTK=OFF
        -DWITH_V4L=OFF -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF
        -DWITH_OBSENSOR=OFF -DWITH_JPEG=OFF -DWITH_PNG=OFF
        -DWITH_WEBP=OFF -DWITH_TIFF=OFF -DWITH_OPENEXR=OFF
        -DWITH_PROTOBUF=OFF -DWITH_FLATBUFFERS=OFF
        -DOPENCV_DOWNLOAD_PATH=<BINARY_DIR>/downloads)
    ExternalProject_Add(vk_opencv_build
        URL "https://github.com/opencv/opencv/archive/refs/tags/${VK_OPENCV_VERSION}.tar.gz"
        URL_HASH "SHA256=${VK_OPENCV_SHA256}"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        PREFIX "${CMAKE_BINARY_DIR}/opencv-external"
        INSTALL_DIR "${_vk_opencv_prefix}"
        CMAKE_ARGS ${_vk_opencv_cmake_args}
        BUILD_BYPRODUCTS
            "${_vk_opencv_prefix}/lib/libopencv_core.a"
            "${_vk_opencv_prefix}/lib/libopencv_imgproc.a"
            "${_vk_opencv_prefix}/lib/libopencv_calib3d.a"
            "${_vk_opencv_prefix}/lib/libopencv_features2d.a"
            "${_vk_opencv_prefix}/lib/libopencv_flann.a"
            "${_vk_opencv_prefix}/lib/libopencv_objdetect.a")
    file(MAKE_DIRECTORY "${_vk_opencv_prefix}/include/opencv4")
endif()

foreach(_vk_excluded IN LISTS VK_OPENCV_EXCLUDED)
    if(EXISTS "${_vk_opencv_prefix}/lib/libopencv_${_vk_excluded}.a")
        message(FATAL_ERROR "Excluded OpenCV module found: ${_vk_excluded}")
    endif()
endforeach()

foreach(_vk_module IN LISTS VK_OPENCV_MODULES)
    if(VISIONKIT_OPENCV_DIR AND NOT EXISTS "${_vk_opencv_prefix}/lib/libopencv_${_vk_module}.a")
        message(FATAL_ERROR "Required OpenCV module missing: ${_vk_module}")
    endif()
    add_library(VisionKit::opencv_${_vk_module} STATIC IMPORTED GLOBAL)
    set_target_properties(VisionKit::opencv_${_vk_module} PROPERTIES
        IMPORTED_LOCATION "${_vk_opencv_prefix}/lib/libopencv_${_vk_module}.a"
        INTERFACE_INCLUDE_DIRECTORIES "${_vk_opencv_prefix}/include/opencv4")
    if(NOT VISIONKIT_OPENCV_DIR)
        add_dependencies(VisionKit::opencv_${_vk_module} vk_opencv_build)
    endif()
endforeach()

set(VK_OPENCV_LIBRARIES
    VisionKit::opencv_objdetect VisionKit::opencv_calib3d
    VisionKit::opencv_features2d VisionKit::opencv_flann
    VisionKit::opencv_imgproc VisionKit::opencv_core)
