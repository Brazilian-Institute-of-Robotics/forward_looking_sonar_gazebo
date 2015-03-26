rock_library(vizkit3d_normal_depth_map
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(vizkit3d_normal_depth_map_bin Main.cpp
    DEPS vizkit3d_normal_depth_map)

