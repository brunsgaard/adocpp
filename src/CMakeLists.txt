SET(headers
  graph.h
)

SET(sources
  graph.cc
)

add_library(compiled OBJECT ${sources} ${headers})
add_executable(test $<TARGET_OBJECTS:compiled> main.cc)
#add_executable(importer importer.cc)
#add_executable(importer importer.cc)
