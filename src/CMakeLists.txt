rock_library(radar_base
    SOURCES Arpa.cpp EchoToImageLUT.cpp Radar.cpp
    HEADERS Arpa.hpp EchoToImageLUT.hpp Radar.hpp
    DEPS_PKGCONFIG ${OPENCV_PACKAGE} base-types)
