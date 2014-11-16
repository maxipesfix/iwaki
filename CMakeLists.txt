cmake_minimum_required(VERSION 2.8)
project(IWAKI)

set(CMAKE_CXX_FLAGS "-g -Wall")

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  #set(BUILD_SHARED_LIBS OFF)
  #set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static -static-libgcc -static-libstdc++")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static-libgcc")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++")
  set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS} -static-libgcc -s")
  set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -static-libgcc -static-libstdc++ -s")
endif()

#set(NCURSES_LIBRARY "/usr/lib/i386-linux-gnu/libncursesw.a")
#set(NCURSES_INCLUDE_PATH "/usr/include")

# exclude RE2 from MinGW installation
if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(RE2_LIBRARY "/usr/local/lib/libre2.a")
	set(RE2_INCLUDE_PATH "/usr/local/include")
	find_package (Threads)
endif()

set(CURSES_USE_NCURSES TRUE)

if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	find_package(Curses)
endif()

add_subdirectory(queueio)
add_subdirectory(iwaki)
add_subdirectory(soundboard/src)
