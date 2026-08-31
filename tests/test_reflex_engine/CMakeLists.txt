# Reflex Engine Google Test Target

add_executable(test_reflex_engine
    test_packml.cpp
)

target_include_directories(test_reflex_engine PRIVATE
    ../../src/reflex_engine/include
)

target_link_libraries(test_reflex_engine PRIVATE
    GTest::gtest_main
    Threads::Threads
)

include(GoogleTest)
gtest_discover_tests(test_reflex_engine)
