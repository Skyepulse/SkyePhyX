cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_FLAGS "/std:c++17" CACHE STRING "" FORCE)

project(SkyePhyX LANGUAGES C CXX)

# Necessary to work for assimp and dawn
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_SOURCE_DIR}/src/*.h
  ${CMAKE_SOURCE_DIR}/src/*.hpp
  ${CMAKE_SOURCE_DIR}/src/*.cpp
)
list(APPEND SRC_FILES main.cpp)

add_executable(SkyePhyX ${SRC_FILES})

include(FetchContent)

# ImGUI
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.92.5
)
FetchContent_MakeAvailable(imgui)

#Eigen
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(eigen)

# CONFIGURATION

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
)

set(DAWN_ROOT "${CMAKE_SOURCE_DIR}/dawn")
set(DAWN_FETCH_DEPENDENCIES ON CACHE BOOL "" FORCE)
set(DAWN_USE_BUILT_DXC ON CACHE BOOL "" FORCE)

add_subdirectory("${DAWN_ROOT}" "${CMAKE_BINARY_DIR}/dawn_build" EXCLUDE_FROM_ALL)

if(EMSCRIPTEN)

  target_compile_definitions(SkyePhyX PRIVATE 
    IMGUI_IMPL_WEBGPU_BACKEND_EMSCRIPTEN
  )

  set(DCMAKE_CXX_SCAN_FOR_MODULES OFF CACHE BOOL "" FORCE) # Disable C++ modules for Emscripten
  set_target_properties(SkyePhyX PROPERTIES SUFFIX ".html")
  target_link_libraries(SkyePhyX PRIVATE emdawnwebgpu_cpp webgpu_glfw)
  target_link_options(SkyePhyX PRIVATE 
      "-sINITIAL_MEMORY=64MB" 
      "-sALLOW_MEMORY_GROWTH=1" 
      "-sASYNCIFY=1" 
      "-sUSE_GLFW=3" 
      "--preload-file=${CMAKE_SOURCE_DIR}/Shaders@/Shaders"
      "--preload-file=${CMAKE_SOURCE_DIR}/web-data@/data"
      "--shell-file=${CMAKE_SOURCE_DIR}/shell.html"
      "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap']"
      "-sMODULARIZE=1"
      "-sEXPORT_NAME='createSkyePhyXModule'"
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