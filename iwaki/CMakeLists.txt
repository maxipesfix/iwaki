include_directories(${IWAKI_SOURCE_DIR}/iwaki)
link_directories(${IWAKI_BINARY_DIR}/iwaki)

include_directories (${IWAKI_SOURCE_DIR}/queueio) 
add_library(iwaki iwaki.cc)
