#/*****************************************************************************
#                                                                            *
# Copyright 2020 AT&T Intellectual Property                                  *
#                                                                            *
# Licensed under the Apache License, Version 2.0 (the "License");            *
# you may not use this file except in compliance with the License.           *
# You may obtain a copy of the License at                                    *
#                                                                            *
#      http://www.apache.org/licenses/LICENSE-2.0                            *
#                                                                            *
# Unless required by applicable law or agreed to in writing, software        *
# distributed under the License is distributed on an "AS IS" BASIS,          *
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
# See the License for the specific language governing permissions and        *
# limitations under the License.                                             *
#                                                                            *
#******************************************************************************/

# For clarity: this generates object, not a lib as the CM command implies.
#

file(GLOB SOURCES "*.cc")
file(GLOB HEADERS "*.h")

# set(SOURCES control_message_encoder_decoder.cc handover_control_message.cc)
# set(HEADERS control_message_encoder_decoder.h handover_control_message.h)

# include_directories(../../asn1c_defs)
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
  include_directories(${e2sim_include_dirs})
endif()

include_directories(.)

# add_library( ric_control_objects OBJECT ${SOURCES})
add_library( ric_control_objects OBJECT ${SOURCES})
target_compile_definitions(ric_control_objects PUBLIC ASN_DISABLE_OER_SUPPORT)

target_include_directories (ric_control_objects PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PUBLIC src)

# header files should go into .../include/xfcpp/
if( DEV_PKG )                                   
  install( FILES
    ${HEADERS}
    DESTINATION ${install_inc}
    )
endif()
# ---------------------------------------------------------------------------
