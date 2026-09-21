rock_library(radar_base
    SOURCES Radar.cpp EchoToImageLUT.cpp
    HEADERS Radar.hpp EchoToImageLUT.hpp BaseTypes.hpp
    DEPS_PKGCONFIG ${OPENCV_PACKAGE} base-types)
