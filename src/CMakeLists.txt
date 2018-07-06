add_executable(opengl_viewer
  viewer.h viewer.cpp
  glad.c opengl_viewer.cpp
)

target_link_libraries(opengl_viewer
  ${OPENGL_gl_LIBRARY} 
  ${GLFW3_LIBRARY} 
  ${ASSIMP_LIBRARIES} 
  ${CMAKE_DL_LIBS}
  ${PCL_LIBRARIES}
)
