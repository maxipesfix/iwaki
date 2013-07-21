cmake_minimum_required(VERSION 2.8)
project(IWAKI)


set(CMAKE_CXX_FLAGS "-g -Wall")
#set(NCURSES_LIBRARY "/usr/lib/i386-linux-gnu/libncursesw.a")
#set(NCURSES_INCLUDE_PATH "/usr/include")

# exclude RE2 from MinGW installation
if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(RE2_LIBRARY "/usr/local/lib/libre2.a")
	set(RE2_INCLUDE_PATH "/usr/local/include/re2")
	find_package (Threads)
endif ()

set(CURSES_USE_NCURSES TRUE)

if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	find_package(Curses)
endif()

add_subdirectory(queueio)
add_subdirectory(iwaki)
add_subdirectory(soundboard/src)