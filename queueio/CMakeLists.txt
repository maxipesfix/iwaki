include_directories(${IWAKI_SOURCE_DIR}/queueio)
link_directories(${IWAKI_BINARY_DIR}/queueio)

add_library(queueio STATIC queueio.cc tinystr.cc tinyxml.cc tinyxmlerror.cc tinyxmlparser.cc)
