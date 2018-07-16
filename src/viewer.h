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
#include "camera.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;



class Viewer{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Viewer();

  int init();
  void processInput();
  bool windowShouldClose();
  void render();
  void update();
  void terminate();
  void loadCloud(char* filename);
  void loadBox();
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
  Camera _camera;

  unsigned int _cVBO,_bVBO;
  unsigned int _cVAO,_bVAO;

  std::vector<float> _cloud_vertices;
  int _num_cloud_vertices;
  std::vector<float> _box_vertices;
  int _num_box_vertices;

private:
  static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
  void mouse_callback(GLFWwindow* window, double xpos, double ypos);

  bool _firstMouse;
  float _deltaTime;
  float _lastFrame;
  float _lastX;
  float _lastY;

  Eigen::Vector3f _min,_max;
  void insertVertex(std::vector<float> &vertices, float x, float y, float z, float r, float g, float b);

};
