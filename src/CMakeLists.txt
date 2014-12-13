#SET(headers
#)

#SET(sources
#dijkstra_demo.cc
#)

#add_library(compiled OBJECT ${sources} ${headers})
#add_executable(dijkstra dijkstra_demo.cc)
add_executable(importer importer.cc)
