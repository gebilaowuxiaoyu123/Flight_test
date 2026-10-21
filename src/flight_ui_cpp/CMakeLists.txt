cmake_minimum_required(VERSION 3.8)
project(flight_ui_cpp CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Boost.Beast 是 header-only，但需要 Boost.System
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(jsoncpp REQUIRED)

# ---------------------------------------------------------------- 核心库
add_library(flight_ui_cpp_core
  src/px4_adapter.cpp
  src/telemetry.cpp
  src/rc_service.cpp
  src/web_server.cpp
)
target_include_directories(flight_ui_cpp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(flight_ui_cpp_core rclcpp px4_msgs)
# ament_target_dependencies 用的是 plain 签名，这里也必须用 plain
# （CMake 不允许同一 target 混用 plain 和 keyword 签名）
target_include_directories(flight_ui_cpp_core SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(flight_ui_cpp_core ${Boost_LIBRARIES} JsonCpp::JsonCpp)

# ---------------------------------------------------------------- 可执行
add_executable(rc_server src/rc_server_main.cpp)
ament_target_dependencies(rc_server ament_index_cpp rclcpp)
target_include_directories(rc_server SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(rc_server flight_ui_cpp_core ${Boost_LIBRARIES} JsonCpp::JsonCpp)

install(TARGETS rc_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# 网页界面只维护一份：直接用 Python 包里那一份，避免两份前端代码漂移
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../flight_ui/web/
  DESTINATION share/${PROJECT_NAME}/web
  FILES_MATCHING PATTERN "*.html" PATTERN "*.js" PATTERN "*.css"
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
