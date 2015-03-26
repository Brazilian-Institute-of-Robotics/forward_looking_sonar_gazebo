rock_testsuite(NormalDepthMap_core NormalDepthMap_test.cpp TestHelper.cpp
    DEPS normal_depth_map
    LIBS ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG opencv)

rock_testsuite(ImageViewerCaptureTool_core ImageViewerCaptureTool_test.cpp
    DEPS normal_depth_map
    LIBS ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG opencv)

rock_testsuite(NormalMapping_core NormalMapping_test.cpp TestHelper.cpp
    DEPS normal_depth_map
    LIBS ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG opencv)

rock_testsuite(MaterialProperties_core MaterialProperties_test.cpp TestHelper.cpp
    DEPS normal_depth_map
    LIBS ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG opencv)

rock_testsuite(Attenuation_core Attenuation_test.cpp TestHelper.cpp
    DEPS normal_depth_map
    LIBS ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG opencv)
