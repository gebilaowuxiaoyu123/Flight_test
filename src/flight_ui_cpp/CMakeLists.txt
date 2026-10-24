cmake_minimum_required(VERSION 3.8)
project(flight_ui_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
# Boost.Beast 是 header-only，但要链 Boost.System / Boost.Thread
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
# JSON 用 jsoncpp（JsonCpp::JsonCpp 会自动带上 /usr/include/jsoncpp）
find_package(jsoncpp REQUIRED)

add_executable(rc_server src/rc_server.cpp)
target_include_directories(rc_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rc_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  rc_server
  "rclcpp"
  "px4_msgs"
  "ament_index_cpp"
)
# 注意：ament_target_dependencies 用的是 plain 签名，
# 这里必须也用 plain（不能写 PRIVATE/PUBLIC），否则 CMake 报
# "must be either all-keyword or all-plain"
target_include_directories(rc_server SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(rc_server ${Boost_LIBRARIES} JsonCpp::JsonCpp)

install(TARGETS rc_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../flight_ui/web/
  DESTINATION share/${PROJECT_NAME}/web
  FILES_MATCHING PATTERN "*.html" PATTERN "*.js" PATTERN "*.css" PATTERN "*.png" PATTERN "*.ico"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
