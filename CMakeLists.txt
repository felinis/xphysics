cmake_minimum_required(VERSION 3.16)
project(xphysics CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES 
	src/xp_broadphase.cxx
	src/xp_narrowphase.cxx
	src/xp_solver.cxx
	src/xp.cxx
)
set(HEADERS
	src/xp_broadphase.hxx
	src/xp_internal.hxx
	src/xp_math.hxx
	src/xp_math_operators.hxx
	src/xp_narrowphase.hxx
	src/xp_solver.hxx
	inc/xp.h
)

add_library(xphysics SHARED ${SOURCES} ${HEADERS})

# public include directory
target_include_directories(xphysics PUBLIC inc)

# define XP_EXPORT so headers use __declspec(dllexport) / visibility attribute
target_compile_definitions(xphysics PRIVATE XP_EXPORT)

# enable AVX support
if(MSVC)
    target_compile_options(xphysics PUBLIC /arch:AVX)
else()
    target_compile_options(xphysics PUBLIC -mavx)
endif()

# unit tests
add_executable(xp_test_math tests/xp_test_math.cxx)
target_link_libraries(xp_test_math PRIVATE xphysics)
target_include_directories(xp_test_math PRIVATE src) # we allow unit tests to access our internal src folder

add_executable(xp_test_main tests/xp_test_main.cxx)
target_link_libraries(xp_test_main PRIVATE xphysics)

# todo: in the future it could be interesting to avoid using libc/msvcrt altogether for maximum portability,
# but I am not sure how much that is feasible so we just do the default thing for now

# create a compile json file for clangd
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
