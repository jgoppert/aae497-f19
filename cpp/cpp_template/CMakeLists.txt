project(cmake_template)
cmake_minimum_required(VERSION 3.10)

enable_testing()

find_package(gazebo)


add_executable(hello_world hello_world.cpp)

add_executable(my_test my_test.cpp)

<<<<<<< HEAD
=======
add_executable(objects objects.cpp)
add_executable(templates templates.cpp)

>>>>>>> eb4087acefb7e3964fbf7138371e79e42431d5e2
add_executable(movies movies.cpp)

# my gazebo plugin
add_library(my_gazebo_plugin SHARED my_gazebo_plugin.cpp)
target_include_directories(my_gazebo_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(my_gazebo_plugin ${GAZEBO_LIBRARIES})

add_test(my_test my_test)
add_test(objects objects)
