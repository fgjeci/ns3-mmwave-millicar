# check_include_file_cxx(stdint.h HAVE_STDINT_H)
# if(HAVE_STDINT_H)
#     add_definitions(-DHAVE_STDINT_H)
# endif()

# set(examples_as_tests_sources)
# if(${ENABLE_EXAMPLES})
#     set(examples_as_tests_sources
#         #test/oran-interface-examples-test-suite.cc
#         )
# endif()

# build_lib(
#     LIBNAME oran-interface
#     SOURCE_FILES model/oran-interface.cc
#                  helper/oran-interface-helper.cc
#     HEADER_FILES model/oran-interface.h
#                  helper/oran-interface-helper.h
#     LIBRARIES_TO_LINK ${libcore}
#     TEST_SOURCES test/oran-interface-test-suite.cc
#                  ${examples_as_tests_sources}
# )
 
set(source_files
    model/oran-interface.cc
    model/asn1c-types.cc
    model/function-description.cc
    model/kpm-indication.cc
    model/kpm-function-description.cc
    model/ric-control-message.cc
    model/ric-control-function-description.cc
    helper/oran-interface-helper.cc
    helper/indication-message-helper.cc
    helper/lte-indication-message-helper.cc
    helper/mmwave-indication-message-helper.cc
    # modified
    # model/handover_ric_control/all_handovers.cc
    # model/handover_ric_control/cell_handovers_list.cc
    # model/handover_ric_control/control_message_encoder_decoder.cc
    # model/handover_ric_control/handover_item.cc
    # model/handover_ric_control/handover_list.cc
    # end modification
)

set(test_sources
    test/oran-interface-test-suite.cc
)

set(header_files
    model/oran-interface.h 
    model/asn1c-types.h
    model/function-description.h
    model/kpm-indication.h
    model/kpm-function-description.h
    model/ric-control-message.h
    model/ric-control-function-description.h
    helper/oran-interface-helper.h
    helper/indication-message-helper.h
    helper/lte-indication-message-helper.h
    helper/mmwave-indication-message-helper.h
    # modified
    # model/handover_ric_control/control_message_encoder_decoder.h 
    # model/handover_ric_control/all_handovers.h
    # model/handover_ric_control/cell_handovers_list.h
    # model/handover_ric_control/handover_item.h
    # model/handover_ric_control/handover_list.h
    # end modification
)

# add_subdirectory(model/handover-ric-control)


find_external_library(DEPENDENCY_NAME e2sim
                    HEADER_NAME e2sim.hpp
                      LIBRARY_NAME e2sim
                      SEARCH_PATHS /usr/local/lib)

# message("TEST  ${e2sim_INCLUDE_DIRS}") 
# message("TEST libraries ${e2sim_LIBRARIES}") 

if(${e2sim_FOUND})
  set(e2sim_libraries ${e2sim_LIBRARIES})
  # target_include_directories(${PROJECT_NAME} PUBLIC ${e2sim_INCLUDE_DIRS})
  set(e2sim_include_dirs ${e2sim_INCLUDE_DIRS})
  # foreach(singlelib ${e2sim_include_dirs})
  #     if((${singlelib} MATCHES "e2sim"))
  #       message("Including  ${singlelib}") 
  #       include_directories(${singlelib})
  #     endif()
  # endforeach()
  include_directories(${e2sim_include_dirs})
  # link_libraries(${e2sim_libraries})
endif()

set(libraries_to_link
    ${libcore}
    ${e2sim_libraries}
)

message("Liberies to link ${e2sim_INCLUDE_DIRS} ${libraries_to_link} ")  


# add_library(e2sim_imp STATIC IMPORTED) # or STATIC instead of SHARED
# set_target_properties(e2sim_imp PROPERTIES
#   IMPORTED_LOCATION "/usr/local/lib/libe2sim.a"
#   INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include/e2sim"
# )

build_lib(
  LIBNAME oran-interface
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
    # ${E2SIM_HEADER_DIR}
  LIBRARIES_TO_LINK
    ${libraries_to_link}
  TEST_SOURCES ${test_sources}
)




