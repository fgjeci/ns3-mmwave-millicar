set(source_files
    model/mmwave-vehicular.cc
    model/mmwave-sidelink-spectrum-phy.cc
    model/mmwave-sidelink-spectrum-signal-parameters.cc
    model/mmwave-sidelink-phy.cc
    model/mmwave-sidelink-mac.cc
    model/mmwave-vehicular-net-device.cc
    model/mmwave-vehicular-antenna-array-model.cc
    helper/mmwave-vehicular-helper.cc
    helper/mmwave-vehicular-traces-helper.cc
    # modified
    model/mmwave-vehicular-5g-net-device.cc
    helper/mmwave-vehicular-5g-helper.cc
    helper/mmwave-vehicular-indication-message-helper.cc
    model/mmwave-packet-relay-tag.cc
    model/mmwave-packet-relay-header.cc
    helper/stats-helper/sinr-report-stats.cc
    helper/stats-helper/stats-helper.cc
    helper/stats-helper/send-packet-stats.cc
    helper/stats-helper/ef-mac-rlc-buffer-status.cc
    helper/parameters-config.cc
    # modified
    model/node-v2v-channel-condition-model.cc
    model/three-gpp-v2v-blockage-channel-condition-model.cc
    model/channel-condition-model-deterministic.cc
    # end modification
)

set(test_sources
    test/mmwave-vehicular-spectrum-phy-test.cc
    test/mmwave-vehicular-rate-test.cc
    test/mmwave-vehicular-interference-test.cc
)

set(header_files
    model/mmwave-vehicular.h
    model/mmwave-sidelink-spectrum-phy.h
    model/mmwave-sidelink-spectrum-signal-parameters.h
    model/mmwave-sidelink-phy.h
    model/mmwave-sidelink-mac.h
    model/mmwave-sidelink-sap.h
    model/mmwave-vehicular-net-device.h
    model/mmwave-vehicular-antenna-array-model.h
    helper/mmwave-vehicular-helper.h
    helper/mmwave-vehicular-traces-helper.h
    # modified
    model/mmwave-vehicular-5g-net-device.h
    helper/mmwave-vehicular-5g-helper.h
    helper/mmwave-vehicular-indication-message-helper.h
    model/mmwave-packet-relay-tag.h
    model/mmwave-packet-relay-header.h
    helper/stats-helper/sinr-report-stats.h
    helper/stats-helper/stats-helper.h
    helper/stats-helper/send-packet-stats.h
    helper/parameters-config.h
    model/node-v2v-channel-condition-model.h
    model/three-gpp-v2v-blockage-channel-condition-model.h
    helper/stats-helper/ef-mac-rlc-buffer-status.h
    model/channel-condition-model-deterministic.h
    # end modification
)

find_external_library(DEPENDENCY_NAME e2sim
                        HEADER_NAME e2sim.hpp
                        LIBRARY_NAME e2sim
                        SEARCH_PATHS /usr/local/lib)

message("The directory found ${e2sim_INCLUDE_DIRS} ${e2sim_FOUND}")

if(${e2sim_FOUND})
  set(e2sim_include_dirs ${e2sim_INCLUDE_DIRS})
  include_directories("${e2sim_include_dirs}")
endif()

build_lib(
  LIBNAME millicar
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libpropagation}
    ${libspectrum}
    ${libmmwave}
    ${liblte}
    ${libbuildings}
    ${libaodv}
    ${libmobility}
    ${libnetwork}
    ${liboran-interface}
  TEST_SOURCES ${test_sources}
)