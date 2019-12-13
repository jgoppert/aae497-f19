project(multiply_two)

cmake_minimum_required(VERSION 2.5)

include_directories(
	.
	${CMAKE_CURRENT_BINARY_DIR}
	)


add_custom_command(OUTPUT func.c func.h COMMAND python ${CMAKE_SOURCE_DIR}/gen.py DEPENDS ${CMAKE_SOURCE_DIR}/gen.py)

add_definitions(
	-Dcasadi_int=int
	-Dcasadi_real=double
	)

add_executable(two
	${CMAKE_CURRENT_BINARY_DIR}/func.c
	two.cpp
	)
