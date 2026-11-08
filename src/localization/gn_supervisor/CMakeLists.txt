cmake_minimum_required(VERSION 3.8)
project(gn_supervisor)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(ament_cmake REQUIRED)

# The Python (flat, run by path: the modules import each other and the node starts search_v2.py as a script) goes to lib/gn_supervisor next to the worker's binaries.
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

option(GN_SUPERVISOR_PYTHON_ONLY "install the Python only; the worker binaries come from another prefix (an overlay of this source over an image: the launch finds lib/gn_supervisor/register_v2 along the ament path)" OFF)
if(NOT GN_SUPERVISOR_PYTHON_ONLY)
  # Where small_gicp comes from. The image keeps two builds: /usr/local is single-threaded (level_corrector must stay a small share of one core), register_v2 wants the TBB one
  # (num_threads = 2 in register_v2.cpp; the binary the team staged by hand before was TBB too). Empty = find_package's default search.
  set(GN_SUPERVISOR_SMALL_GICP_DIR "" CACHE PATH "directory of small_gicpConfig.cmake for register_v2")
  if(GN_SUPERVISOR_SMALL_GICP_DIR)
    find_package(small_gicp REQUIRED CONFIG PATHS ${GN_SUPERVISOR_SMALL_GICP_DIR} NO_DEFAULT_PATH)
  else()
    find_package(small_gicp REQUIRED CONFIG)
  endif()
  find_package(Eigen3 REQUIRED)
  find_package(Threads REQUIRED)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)   # the installed register_v2 finds the libsmall_gicp.so it was linked against

  # libcorr_search.so: the hot loop of the correlative search, a C API for ctypes (scripts/corr_cpp.py). -O3 without -march (one binary for every Orin) and without NDEBUG,
  # as the g++ line this replaces; pocketfft (BSD-3) is header-only in third_party/.
  add_library(corr_search SHARED src/corr_search.cpp)
  target_compile_features(corr_search PRIVATE cxx_std_17)
  target_include_directories(corr_search PRIVATE third_party/pocketfft)
  target_compile_options(corr_search PRIVATE -O3 -UNDEBUG)
  target_link_libraries(corr_search PRIVATE Threads::Threads)

  # register_v2: GICP from every candidate pose (see the header of src/register_v2.cpp).
  add_executable(register_v2 src/register_v2.cpp)
  target_compile_features(register_v2 PRIVATE cxx_std_17)
  target_link_libraries(register_v2 PRIVATE small_gicp::small_gicp Eigen3::Eigen)

  install(TARGETS corr_search LIBRARY DESTINATION lib/${PROJECT_NAME})
  install(TARGETS register_v2 RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()
ament_package()
