cmake_minimum_required(VERSION 2.8)
project(IWAKI)


set(CMAKE_CXX_FLAGS "-g -Wall")
#set(NCURSES_LIBRARY "/usr/lib/i386-linux-gnu/libncursesw.a")
#set(NCURSES_INCLUDE_PATH "/usr/include")
#set(RE2_LIBRARY "/usr/local/lib/libre2.a")
# testing
set(RE2_LIBRARY "c:/Users/maxim/Downloads/re2-2011-09-30-src-win32/re2/Release/re2.lib")
set(RE2_INCLUDE_PATH "/usr/local/include/re2")
find_package (Threads)

set(CURSES_USE_NCURSES TRUE)
find_package(Curses)

add_subdirectory(queueio)
add_subdirectory(iwaki)
add_subdirectory(soundboard/src)