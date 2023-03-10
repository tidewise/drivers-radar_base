rock_library(radar_base
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(radar_base_bin Main.cpp
    DEPS radar_base)

