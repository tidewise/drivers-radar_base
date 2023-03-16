rock_library(radar_base
    SOURCES Radar.cpp
    HEADERS Radar.hpp
    DEPS_PKGCONFIG base-types)

rock_executable(radar_base_bin Main.cpp
    DEPS radar_base )

