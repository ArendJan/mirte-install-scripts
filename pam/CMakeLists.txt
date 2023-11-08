cmake_minimum_required(VERSION 3.10)
project(mirte_pam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

find_package(PAM)

# Check if we found PAM.
if (NOT PAM_FOUND)
    message(FATAL_ERROR "PAM library was not found. Install: sudo apt-get install libpam0g-dev")
endif ()

include_directories(
   ${PAM_INCLUDE_DIR}
   ${CMAKE_BINARY_DIR}
   ${CMAKE_CURRENT_BINARY_DIR}
)
find_package(json-c)
if(NOT json-fc_FOUND)
  message("Json-c not found, installing it for you!")
  execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/install_json.sh)
  message("Done installing json-c")

  find_package(json-c)
  if(NOT json-c_FOUND)
  message(FATAL_ERROR "Tried installing json-c, still not found")
  endif()
endif()
# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

set(modules warn storepassword)

foreach(module ${modules})
    add_library(mirte_pam_${module} SHARED ${module}.c)
    
    # find_package(json-c CONFIG)
    target_link_libraries(mirte_pam_${module} PRIVATE json-c::json-c)

    install(TARGETS mirte_pam_${module} DESTINATION /lib/security/)

endforeach(module ${modules})

add_executable(myprogram test.c storepassword.c)
target_link_libraries(myprogram PRIVATE json-c::json-c)
install(CODE "execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/install_module.sh)")