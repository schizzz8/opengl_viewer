#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>

typedef pcl::PointXYZ Point;
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

  int loadCloud(char* filename);

protected:
  int _w;
  int _h;

  GLFWwindow* _window;

  PointCloud::Ptr _cloud;

private:

  //viewport size callback
  static void framebuffer_size_callback(GLFWwindow* window, int width, int height);

};