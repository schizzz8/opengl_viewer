#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>

#include "shader.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class Viewer{
public:
  Viewer();

  int init();
  void processInput();
  bool windowShouldClose();
  void render();
  void update();
  void terminate();
  void loadCloud(char* filename);
  void generateVertices();
  void bindBuffers();

protected:
  int _w;
  int _h;

  glm::mat4 _model;
  glm::mat4 _view;
  glm::mat4 _projection;

  GLFWwindow* _window;

  Shader _shader;

  std::vector<float> _vertices;
  int _num_vertices;
  unsigned int _VBO;
  unsigned int _VAO;

private:
  static void framebuffer_size_callback(GLFWwindow* window, int width, int height);

};
