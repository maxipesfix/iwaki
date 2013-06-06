include_directories(${IWAKI_SOURCE_DIR}/iwaki)
link_directories(${IWAKI_BINARY_DIR}/iwaki)

include_directories (${IWAKI_SOURCE_DIR}/queueio ${RE2_INCLUDE_PATH}) 
link_directories(${IWAKI_BINARY_DIR}/queueio)

add_library(iwaki STATIC iwaki.cc exparser.cc formula.cc outputs.cc recipe.cc)
target_link_libraries(iwaki queueio ${RE2_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

install (TARGETS iwaki DESTINATION lib)
install (FILES iwaki.h tree.hh DESTINATION include)