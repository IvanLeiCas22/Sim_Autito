# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "CMakeFiles\\Sim_Autito_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\Sim_Autito_autogen.dir\\ParseCache.txt"
  "Sim_Autito_autogen"
  )
endif()
