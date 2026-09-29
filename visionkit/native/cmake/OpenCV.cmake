include_guard(GLOBAL)
include(ExternalProject)

set(VK_OPENCV_VERSION 4.14.0)
set(VK_OPENCV_SHA256 ee8fb9b30eb60850431b4656447080e3737b56e45719c92b67f245950609f86e)
set(VK_OPENCV_MODULES core imgproc calib3d features2d flann objdetect)
set(VK_OPENCV_EXCLUDED dnn highgui videoio imgcodecs gapi ml photo stitching python2 python3 java)

if(NOT VISIONKIT_OPENCV_DIR AND DEFINED ENV{VISIONKIT_OPENCV_DIR})
    set(VISIONKIT_OPENCV_DIR "$ENV{VISIONKIT_OPENCV_DIR}")
endif()

if(VISIONKIT_OPENCV_DIR)
    set(_vk_opencv_prefix "${VISIONKIT_OPENCV_DIR}")
    if(NOT EXISTS "${_vk_opencv_prefix}/include/opencv4/opencv2/core.hpp")
        message(FATAL_ERROR "VISIONKIT_OPENCV_DIR must be an installed OpenCV prefix")
    endif()
    if(NOT EXISTS "${_vk_opencv_prefix}/lib/cmake/opencv4/OpenCVModules.cmake")
        message(FATAL_ERROR "VISIONKIT_OPENCV_DIR lacks OpenCV's exported link targets")
    endif()
else()
    # Each CMake build owns its dependency. Shared installs are opt-in only.
    set(_vk_opencv_prefix "${CMAKE_CURRENT_BINARY_DIR}/opencv-install")
    set(_vk_opencv_cmake_args
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        -DCMAKE_INSTALL_LIBDIR=lib
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
        -DWITH_EIGEN=OFF -DWITH_LAPACK=OFF
        -DWITH_CAROTENE=OFF -DWITH_KLEIDICV=OFF
        -DOPENCV_DOWNLOAD_PATH=<BINARY_DIR>/downloads)
    ExternalProject_Add(vk_opencv_build
        URL "https://github.com/opencv/opencv/archive/refs/tags/${VK_OPENCV_VERSION}.tar.gz"
        URL_HASH "SHA256=${VK_OPENCV_SHA256}"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        PREFIX "${CMAKE_CURRENT_BINARY_DIR}/opencv-external"
        INSTALL_DIR "${_vk_opencv_prefix}"
        CMAKE_ARGS ${_vk_opencv_cmake_args}
        BUILD_BYPRODUCTS
            "${_vk_opencv_prefix}/lib/libopencv_core.a"
            "${_vk_opencv_prefix}/lib/libopencv_imgproc.a"
            "${_vk_opencv_prefix}/lib/libopencv_calib3d.a"
            "${_vk_opencv_prefix}/lib/libopencv_features2d.a"
            "${_vk_opencv_prefix}/lib/libopencv_flann.a"
            "${_vk_opencv_prefix}/lib/libopencv_objdetect.a")
    ExternalProject_Add_Step(vk_opencv_build check_modules
        COMMAND "${CMAKE_COMMAND}" -DPREFIX=${_vk_opencv_prefix}
            -P "${CMAKE_CURRENT_LIST_DIR}/CheckOpenCVModules.cmake"
        DEPENDEES install ALWAYS 1)
    ExternalProject_Add_StepTargets(vk_opencv_build check_modules)
    file(MAKE_DIRECTORY "${_vk_opencv_prefix}/include/opencv4")
endif()

# Check prebuilt prefixes now; a fresh ExternalProject is checked after install.
if(VISIONKIT_OPENCV_DIR)
    set(PREFIX "${_vk_opencv_prefix}")
    include("${CMAKE_CURRENT_LIST_DIR}/CheckOpenCVModules.cmake")
    unset(PREFIX)
    # Installed OpenCV exports its complete transitive link interface. The
    # existing shared v2 install includes Eigen, while new builds disable it.
    file(READ "${_vk_opencv_prefix}/lib/cmake/opencv4/OpenCVModules.cmake"
        _vk_opencv_exports)
    if(_vk_opencv_exports MATCHES "Eigen3::Eigen")
        find_package(Eigen3 REQUIRED)
    endif()
    find_package(OpenCV ${VK_OPENCV_VERSION} EXACT CONFIG REQUIRED
        COMPONENTS ${VK_OPENCV_MODULES}
        PATHS "${_vk_opencv_prefix}" NO_DEFAULT_PATH)
    set(VK_OPENCV_LIBRARIES
        opencv_objdetect opencv_calib3d opencv_features2d
        opencv_flann opencv_imgproc opencv_core)
else()
    find_package(Threads REQUIRED)
    foreach(_vk_module IN LISTS VK_OPENCV_MODULES)
        add_library(VisionKit::opencv_${_vk_module} STATIC IMPORTED GLOBAL)
        set_target_properties(VisionKit::opencv_${_vk_module} PROPERTIES
            IMPORTED_LOCATION "${_vk_opencv_prefix}/lib/libopencv_${_vk_module}.a"
            INTERFACE_INCLUDE_DIRECTORIES "${_vk_opencv_prefix}/include/opencv4")
        add_dependencies(VisionKit::opencv_${_vk_module} vk_opencv_build-check_modules)
    endforeach()
    set(VK_OPENCV_LIBRARIES
        VisionKit::opencv_objdetect VisionKit::opencv_calib3d
        VisionKit::opencv_features2d VisionKit::opencv_flann
        VisionKit::opencv_imgproc VisionKit::opencv_core
        Threads::Threads ${CMAKE_DL_LIBS})
    if(UNIX)
        list(APPEND VK_OPENCV_LIBRARIES m)
        if(NOT APPLE)
            list(APPEND VK_OPENCV_LIBRARIES rt)
        endif()
    endif()
endif()
