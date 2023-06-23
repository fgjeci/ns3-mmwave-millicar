set(libs "${libs_to_build}")

# Process subdirectories
foreach(libname ${libs})
  if(EXISTS ${PROJECT_SOURCE_DIR}/contrib/${libname}/CMakeLists.txt)
    message(STATUS "Processing contrib/${libname}")
    add_subdirectory(${libname})
  else()
    message(
      ${HIGHLIGHTED_STATUS}
      "Skipping contrib/${libname} : it does not contain a CMakeLists.txt file"
    )
  endif()
endforeach()

# add_subdirectory("oran-interface")

# add_subdirectory("contrib/oran-interface")
# add_subdirectory("millicar")