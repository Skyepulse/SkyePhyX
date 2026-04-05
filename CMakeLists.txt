cmake_minimum_required(VERSION 3.25)

project(SkyePhyX LANGUAGES C CXX)

# Abseil checks CMAKE_CXX_FLAGS directly for the standard on MSVC.
# Must be CACHE FORCE so it propagates into all subdirectories.
if(MSVC)
    set(CMAKE_CXX_FLAGS "/std:c++17" CACHE STRING "" FORCE)
else()
    set(CMAKE_CXX_STANDARD 17 CACHE STRING "" FORCE)
    set(CMAKE_CXX_STANDARD_REQUIRED ON CACHE BOOL "" FORCE)
endif()

# Necessary to work for assimp and dawn
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# ============================================
# Platform-specific compiler flags
# ============================================
if(MSVC)
    add_compile_options(/FS)
elseif(EMSCRIPTEN)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "" FORCE)
    set(CMAKE_C_FLAGS_RELEASE   "-O3 -DNDEBUG" CACHE STRING "" FORCE)
endif()

# ============================================
# Sources
# ============================================
file(GLOB_RECURSE SRC_FILES
  ${CMAKE_SOURCE_DIR}/src/*.h
  ${CMAKE_SOURCE_DIR}/src/*.hpp
  ${CMAKE_SOURCE_DIR}/src/*.cpp
)
list(APPEND SRC_FILES main.cpp)

add_executable(SkyePhyX ${SRC_FILES})

# ============================================
# Target-specific optimization flags (NOT applied to Dawn)
# ============================================
if(MSVC)
    target_compile_options(SkyePhyX PRIVATE
        $<$<CONFIG:Release>:/O2 /Ob3 /GL /fp:fast /arch:AVX2>
        $<$<CONFIG:RelWithDebInfo>:/O2 /Ob3 /GL /fp:fast /arch:AVX2 /Zi>
    )
    target_link_options(SkyePhyX PRIVATE
        $<$<CONFIG:Release>:/LTCG /OPT:REF /OPT:ICF>
        $<$<CONFIG:RelWithDebInfo>:/LTCG /DEBUG /OPT:REF /OPT:ICF>
    )
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND NOT EMSCRIPTEN)
    target_compile_options(SkyePhyX PRIVATE
        $<$<CONFIG:Release>:-O3 -march=native -ffast-math -flto>
        $<$<CONFIG:RelWithDebInf  o>:-O3 -march=native -ffast-math -flto -g>
    )
    target_link_options(SkyePhyX PRIVATE
        $<$<CONFIG:Release>:-flto>
        $<$<CONFIG:RelWithDebInfo>:-flto>
    )
endif()

# ============================================
# Eigen optimization defines (all platforms)
# ============================================
target_compile_definitions(SkyePhyX PRIVATE
    $<$<NOT:$<CONFIG:Debug>>:NDEBUG>
    $<$<NOT:$<CONFIG:Debug>>:EIGEN_NO_DEBUG>
    EIGEN_DONT_PARALLELIZE
    $<$<NOT:$<CONFIG:Debug>>:EIGEN_FAST_MATH=1>
)

# ============================================
# Dependencies
# ============================================
include(FetchContent)

# QHull still declares cmake_minimum_required(VERSION 3.0).
# CMake 4.x removed compatibility with policy versions below 3.5,
# so we raise the minimum policy compatibility for fetched subprojects.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
  set(CMAKE_POLICY_VERSION_MINIMUM "3.5" CACHE STRING "" FORCE)
endif()

# ImGUI
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.92.5
)
FetchContent_MakeAvailable(imgui)

# Eigen
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(eigen)

# QHull
FetchContent_Declare(
  qhull
  GIT_REPOSITORY https://github.com/qhull/qhull.git
  GIT_TAG        v8.1-alpha1
)
FetchContent_GetProperties(qhull)
if(NOT qhull_POPULATED)
  FetchContent_Populate(qhull)

  # QHull declares a top-level custom target named `uninstall`, which
  # collides with Eigen's `uninstall` target in the same superbuild.
  set(QHULL_CMAKELISTS "${qhull_SOURCE_DIR}/CMakeLists.txt")
  file(READ "${QHULL_CMAKELISTS}" QHULL_CMAKE_CONTENTS)
  string(REPLACE "add_custom_target(uninstall"
                 "add_custom_target(qhull_uninstall"
                 QHULL_CMAKE_CONTENTS
                 "${QHULL_CMAKE_CONTENTS}")
  file(WRITE "${QHULL_CMAKELISTS}" "${QHULL_CMAKE_CONTENTS}")

  add_subdirectory("${qhull_SOURCE_DIR}" "${qhull_BINARY_DIR}" EXCLUDE_FROM_ALL)
endif()

# ============================================
# ImGUI configuration
# ============================================
set(IMGUI_DIR ${imgui_SOURCE_DIR})
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
)
set(IMGUI_BACKEND_SOURCES
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_wgpu.cpp
)
target_sources(SkyePhyX PRIVATE 
    ${IMGUI_SOURCES} 
    ${IMGUI_BACKEND_SOURCES}
)
target_include_directories(SkyePhyX PRIVATE 
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${CMAKE_SOURCE_DIR}/includes
    ${eigen_SOURCE_DIR}
    ${qhull_SOURCE_DIR}/src
)

# QHull headers live under src/libqhull_r and src/libqhullcpp.
# Link the C++ wrapper before the re-entrant core library.
if(TARGET qhullcpp AND TARGET qhullstatic_r)
  target_link_libraries(SkyePhyX PRIVATE qhullcpp qhullstatic_r)
elseif(TARGET qhullcpp AND TARGET qhull_r)
  target_link_libraries(SkyePhyX PRIVATE qhullcpp qhull_r)
else()
  message(FATAL_ERROR "QHull targets were fetched, but qhullcpp with a re-entrant library target was not found.")
endif()

# ============================================
# Dawn
# ============================================
set(DAWN_ROOT "${CMAKE_SOURCE_DIR}/dawn")
set(DAWN_FETCH_DEPENDENCIES ON CACHE BOOL "" FORCE)
set(DAWN_USE_BUILT_DXC ON CACHE BOOL "" FORCE)

add_subdirectory("${DAWN_ROOT}" "${CMAKE_BINARY_DIR}/dawn_build" EXCLUDE_FROM_ALL)

# ============================================
# Platform targets
# ============================================
if(EMSCRIPTEN)

  target_compile_definitions(SkyePhyX PRIVATE 
    IMGUI_IMPL_WEBGPU_BACKEND_EMSCRIPTEN
  )

  set(DCMAKE_CXX_SCAN_FOR_MODULES OFF CACHE BOOL "" FORCE)
  set_target_properties(SkyePhyX PROPERTIES SUFFIX ".html")
  target_link_libraries(SkyePhyX PRIVATE emdawnwebgpu_cpp webgpu_glfw)
  target_link_options(SkyePhyX PRIVATE
      "-sINITIAL_MEMORY=64MB"
      "-sALLOW_MEMORY_GROWTH=1"
      "-sASYNCIFY=1"
      "-sASYNCIFY_STACK_SIZE=16384"
      "-sUSE_GLFW=3"
      "--preload-file=${CMAKE_SOURCE_DIR}/web-data@/data"
      "--shell-file=${CMAKE_SOURCE_DIR}/webSpecifics/shell.html"
      "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap']"
      "-sMODULARIZE=1"
      "-sEXPORT_NAME=createSkyePhyXModule"
  )
  add_custom_command(TARGET SkyePhyX POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_SOURCE_DIR}/webSpecifics/report.pdf
          ${CMAKE_BINARY_DIR}/report.pdf
      COMMENT "Copying report.pdf to build output"
  )

else()

  target_compile_definitions(SkyePhyX PRIVATE 
    IMGUI_IMPL_WEBGPU_BACKEND_DAWN
  )

  target_link_libraries(
        SkyePhyX
        PRIVATE
      webgpu_dawn
      webgpu_glfw
      glfw
  )
  
  add_custom_target(copy_shaders ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/Shaders
      $<TARGET_FILE_DIR:SkyePhyX>/Shaders
    COMMENT "Copying shaders to output directory"
  )
  add_dependencies(copy_shaders SkyePhyX)

endif()

if(NOT EMSCRIPTEN)
  find_package(Threads REQUIRED)
  target_link_libraries(SkyePhyX PRIVATE Threads::Threads)
endif()

# ============================================
# Build type reminder
# ============================================
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified — defaulting to Release")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
