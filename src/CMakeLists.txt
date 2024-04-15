
set(src
Node.cpp
)

include_directories(
    .
)

add_executable(${PROJECT_NAME} ${src})

target_link_libraries(${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    OpenMP::OpenMP_CXX
    glog
)

target_compile_options(${PROJECT_NAME} PUBLIC -Xpreprocessor -fopenmp)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION bin
)

set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)
