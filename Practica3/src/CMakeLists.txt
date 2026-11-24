# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
  ../src/simple_dwa.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/simple_dwa.h
)


LIST(APPEND LIBS ${LIBS})

add_compile_options(
    -O3 
    -march=native 
    -flto=auto
    -fno-plt
)
add_definitions(-DNDEBUG -fmax-errors=1)
add_link_options(-flto=auto)

