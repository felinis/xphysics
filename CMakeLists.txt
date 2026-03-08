cmake_minimum_required(VERSION 3.16)
project(xphysics C)

# ensure C11 standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS FALSE)

set(SOURCES 
	src/xp.c
)
set(HEADERS
	inc/xp.h
)

add_library(xphysics SHARED ${SOURCES} ${HEADERS})

# public include directory
target_include_directories(xphysics PUBLIC inc)

# define XP_EXPORT so headers use __declspec(dllexport) / visibility attribute
target_compile_definitions(xphysics PRIVATE XP_EXPORT)

# todo: in the future it could be interesting to avoid using libc/msvcrt altogether for maximum portability,
# but I am not sure how much that is feasible so we just do the default thing for now

# create a compile json file for clangd
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
