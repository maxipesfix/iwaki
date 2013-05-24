include_directories(${IWAKI_SOURCE_DIR}/queueio)
link_directories(${IWAKI_BINARY_DIR}/queueio)

add_library(queueio queueio.cc)
