include_directories(${IWAKI_SOURCE_DIR}/queueio)
link_directories(${IWAKI_BINARY_DIR}/queueio)

add_library(queueio STATIC queueio.cc tinystr.cc tinyxml.cc tinyxmlerror.cc tinyxmlparser.cc)

install (TARGETS queueio DESTINATION lib)
install (FILES queueio.h DESTINATION include)
install (FILES nlpw_utils.h log.h tinyxml.h tinystr.h DESTINATION include)
