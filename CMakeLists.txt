cmake_minimum_required(VERSION 3.13)

if(ESP_PLATFORM)
  file(GLOB_RECURSE BMP280_DEV
    "src/*.*"
  )

  idf_component_register(
    SRCS ${BMP280_DEV} 
    INCLUDE_DIRS src 
    REQUIRES arduino Adafruit_BusIO SensorCommon
  )

  return()
endif()
