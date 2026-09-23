cmake_minimum_required(VERSION 3.16)
project(leju-keyboard-controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_executable(motor_keyboard_ctrl motor_keyboard_ctrl.cpp)
target_link_libraries(motor_keyboard_ctrl
    lejusdk-lowlevel
)

add_executable(motor_test_runner motor_test_runner.cpp)
target_link_libraries(motor_test_runner
    lejusdk-lowlevel
)

install(TARGETS motor_keyboard_ctrl motor_test_runner
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    add_executable(test_keyboard_ctrl tests/test_keyboard_ctrl.cpp)
    target_link_libraries(test_keyboard_ctrl
        GTest::gtest
        GTest::gtest_main
        lejusdk-utils
    )
    add_test(NAME test_keyboard_ctrl COMMAND test_keyboard_ctrl)

    install(TARGETS test_keyboard_ctrl
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/tests
    )
endif()
