SET(headers
  graph.h
  utils.h
  dfs.h
  dijkstra.h
  ado.h
  sample.h
)

SET(sources
  graph.cc
  dfs.cc
  dijkstra.cc
  ado.cc
  sample.cc
)

add_library(compiled OBJECT ${sources} ${headers})
add_executable(prepro $<TARGET_OBJECTS:compiled> preprocess.cc)
target_link_libraries(prepro ${link_libs})
add_executable(test $<TARGET_OBJECTS:compiled> test.cc)
target_link_libraries(test ${link_libs})
