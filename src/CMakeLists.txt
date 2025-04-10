set(libs "${libs_to_build}")

find_external_library(DEPENDENCY_NAME e2sim
                    HEADER_NAME e2sim.hpp
                      LIBRARY_NAME e2sim
                      SEARCH_PATHS /usr/local/lib)

message("The value in src of ${e2sim_INCLUDE_DIRS}")

if(${e2sim_FOUND})
  set(e2sim_libraries ${e2sim_LIBRARIES})
  set(e2sim_include_dirs ${e2sim_INCLUDE_DIRS})
  # target_include_directories(libmmwave PUBLIC ${e2sim_include_dirs})
  include_directories(${e2sim_include_dirs})
  link_libraries(${e2sim_libraries})
endif()

# Process the visualizer module first if enabled
if(visualizer IN_LIST libs)
  if(${ENABLE_VISUALIZER})
    message(STATUS "Processing src/visualizer")
    add_subdirectory(visualizer)
  endif()
  list(REMOVE_ITEM libs visualizer)
endif()

# Process subdirectories
foreach(libname ${libs})
  if(EXISTS ${PROJECT_SOURCE_DIR}/src/${libname}/CMakeLists.txt)
    message(STATUS "Processing src/${libname}")
    add_subdirectory(${libname})
  else()
    message(
      ${HIGHLIGHTED_STATUS}
      "Skipping src/${libname} : it does not contain a CMakeLists.txt file"
    )
  endif()
endforeach()

# Prevents link errors due to symbol collisions if the same library is linked
# multiple times
list(REMOVE_DUPLICATES ns3-external-libs)

# Build the lib-ns3-static (ns3.x-static-buildtype.a/.lib) with all sublibraries
if(${NS3_STATIC})
  add_library(
    ${lib-ns3-static} STATIC ${PROJECT_SOURCE_DIR}/build-support/empty.cc
                             "${lib-ns3-static-objs}"
  )

  # Replace shared library suffix and check if static version exists before
  # linking
  set(ns3-external-static-libs)
  foreach(sharedlib ${ns3-external-libs})
    if(NOT (${sharedlib} MATCHES ".so"))
      list(APPEND ns3-external-static-libs ${sharedlib})
      continue()
    endif()

    string(REPLACE ".so" ".a" output ${sharedlib})
    if(EXISTS ${output})
      list(APPEND ns3-external-static-libs ${output})
    else()
      message(
        FATAL_ERROR "Static library version of ${sharedlib} was not found"
      )
    endif()
  endforeach()

  # Required by some static libraries, such as sqlite, for some odd reason
  if(LINUX)
    list(APPEND ns3-external-static-libs -ldl)
  endif()

  target_link_libraries(
    ${lib-ns3-static} ${STATIC_LINK_FLAGS} ${LIB_AS_NEEDED_PRE_STATIC}
    ${ns3-external-static-libs} ${LIB_AS_NEEDED_POST_STATIC}
  )
  if(${NS3_CLANG_TIMETRACE})
    add_dependencies(timeTraceReport ${lib-ns3-static})
  endif()
endif()

# Build the lib-ns3 (ns3.x-monolib-buildtype.dll/.dylib/.so) with all
# sublibraries
if(${NS3_MONOLIB})
  add_library(
    ${lib-ns3-monolib} SHARED ${PROJECT_SOURCE_DIR}/build-support/empty.cc
                              "${lib-ns3-static-objs}"
  )
  set_target_properties(
    ${lib-ns3-monolib}
    PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE
               RUNTIME_OUTPUT_DIRECTORY
               ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} # set output directory for DLLs
  )
  target_link_libraries(
    ${lib-ns3-monolib} ${LIB_AS_NEEDED_PRE} ${ns3-external-libs}
    ${LIB_AS_NEEDED_POST}
  )
  if(${NS3_CLANG_TIMETRACE})
    add_dependencies(timeTraceReport ${lib-ns3-monolib})
  endif()
endif()

if(${NS3_FETCH_OPTIONAL_COMPONENTS})
  add_dependency_to_optional_modules_dependencies()
endif()
