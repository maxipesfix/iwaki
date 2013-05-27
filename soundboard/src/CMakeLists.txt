#include_directories(${IWAKI_SOURCE_DIR}/soundboard)
#link_directories(${IWAKI_BINARY_DIR}/soundboard)

include_directories (${IWAKI_SOURCE_DIR}/queueio ${IWAKI_SOURCE_DIR}/iwaki ${CURSES_INCLUDE_PATH} ${RE2_INCLUDE_PATH}) 


link_directories (${IWAKI_BINARY_DIR}/queueio ${IWAKI_BINARY_DIR}/iwaki /usr/local/lib /usr/lib/i386-linux-gnu) 

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/soundboard/bin)
add_executable(soundboard soundboard.cc text_ui.cc generator.cc soundboard.h)
target_link_libraries(soundboard iwaki queueio ${RE2_LIBRARY} ${CURSES_LIBRARY} )
