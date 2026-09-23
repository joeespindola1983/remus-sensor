cmake_minimum_required(VERSION 3.16)
project(remus_proto2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(remus-proto2
  src/main.cpp
  src/app/RemusPiApp.cpp
  src/drivers/LinuxMpu6050.cpp
  src/drivers/LinuxNeo6m.cpp
  src/drivers/LinuxSessionStorage.cpp
  src/drivers/LinuxSsd1306.cpp
  ../../lib/remus-core/src/LiveSpmEstimator.cpp
)

target_include_directories(remus-proto2 PRIVATE
  include
  ../../lib/remus-core/include
)

target_compile_options(remus-proto2 PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(remus-proto2 PRIVATE pthread atomic)

install(TARGETS remus-proto2 RUNTIME DESTINATION bin)
