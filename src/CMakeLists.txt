SET(headers
  graph.h
  utils.h
)

SET(sources
  graph.cc
)

add_library(compiled OBJECT ${sources} ${headers})
add_executable(test $<TARGET_OBJECTS:compiled> main.cc)
target_link_libraries(test ${link_libs})
