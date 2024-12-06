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

set(modules warn storepassword)

foreach(module ${modules})
    add_library(mirte_pam_${module} SHARED ${module}.c)
    
    install(TARGETS mirte_pam_${module} DESTINATION /lib/security/)

endforeach(module ${modules})

add_executable(myprogram test.c storepassword.c)
install(CODE "execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/install_module.sh)")