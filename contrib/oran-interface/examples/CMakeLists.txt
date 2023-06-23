set(base_examples
    # e2sim-integration-example
    # oran-interface-example
    # ric-indication-messages
    # ric-control-function-desc
    # l3-rrc-example
    # test-wrappers
    # ric-control-message
)

include_directories(${E2SIM_HEADER_DIR})

foreach(
  example
  ${base_examples}
)
  build_lib_example(
    NAME ${example}
    SOURCE_FILES ${example}.cc
    LIBRARIES_TO_LINK 
      ${liboran-interface}
      ${e2sim_libraries}
  )
endforeach()
