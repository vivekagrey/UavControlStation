#cmake_minimum_required(VERSION 3.0.0)
#project(GCS VERSION 0.1.0)

#include(CTest)
#enable_testing()

#add_executable(GCS main.cpp)

#set(CPACK_PROJECT_NAME ${PROJECT_NAME})
#set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
#include(CPack)

cmake_minimum_required(VERSION 3.10.2)

# Specify your project's name
project(GCS VERSION 0.1.0)

# Specify at least C++11, better C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable strict handling of warnings
add_definitions("-Wall -Wextra -Werror")

# Finds MAVSDK when installed system wide.
find_package(MAVSDK REQUIRED)

# Specify your app's executable name, and list of source files used to create it.
add_executable(GCS
    main.cpp
    # ... any other source files
)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(GCS
    MAVSDK::mavsdk             #All apps link against mavsdk library
    MAVSDK::mavsdk_action      # If action plugin used by app ...
    MAVSDK::mavsdk_telemetry   #If telemetry plugin used by app ...
    MAVSDK::mavsdk_mission
    MAVSDK::mavsdk_log_files
    # ... Any other linked libraries
)
