#include "viewer.h"

Viewer::Viewer():
  _shader("../shaders/point.vs", "../shaders/point.fs"){
  _w = 800;
  _h = 600;

  _window = 0;


}

int Viewer::init(){
  //[GLFW] initialize
  glfwInit();

  //[GLFW] configure
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  //[GLFW] define window
  _window = glfwCreateWindow(_w, _h, "OpenGL Viewer", NULL, NULL);
  if (_window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(_window);

  //[GLAD] init
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  //[OPENGL] set viewport
  glViewport(0, 0, _w, _h);
  glfwSetFramebufferSizeCallback(_window, Viewer::framebuffer_size_callback);
}

void Viewer::processInput() {
  if(glfwGetKey(_window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(_window, true);
}

bool Viewer::windowShouldClose(){
  return glfwWindowShouldClose(_window);
}

void Viewer::render(){

  //1) clear color buffer
  glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  //2) set shaders to use
  _shader.use();

  //3) bind VAO
  glBindVertexArray(_VAO);

  //4) draw
  glDrawArrays(GL_POINTS,0,3);

}

void Viewer::update(){
  glfwSwapBuffers(_window);
  glfwPollEvents();
}

void Viewer::terminate(){
  //[GLFW] terminate
  glfwTerminate();
}

int Viewer::loadCloud(char *filename){
  //[PCL] load cloud
  PointCloud::Ptr cloud_in (new PointCloud);
  if (pcl::io::loadPCDFile<Point> (filename, *cloud_in) == -1) {
    PCL_ERROR ("Couldn't read file!\n");
    return (-1);
  }

  //[PCL] normalize cloud
  float min_x=std::numeric_limits<float>::max(),min_y=std::numeric_limits<float>::max(),min_z=std::numeric_limits<float>::max();
  float max_x=-std::numeric_limits<float>::max(),max_y=-std::numeric_limits<float>::max(),max_z=-std::numeric_limits<float>::max();

  for(size_t i=0; i<cloud_in->size(); ++i){
    const Point& point = cloud_in->at(i);

    if(point.x < min_x)
      min_x = point.x;
    if(point.x > max_x)
      max_x = point.x;

    if(point.y < min_y)
      min_y = point.y;
    if(point.y > max_y)
      max_y = point.y;

    if(point.z < min_z)
      min_z = point.z;
    if(point.z > max_z)
      max_z = point.z;
  }
  float x_range = max_x-min_x;
  float y_range = max_y-min_y;
  float z_range = max_z-min_z;

  _vertices.resize(cloud_in->size()*3);
  for(size_t i=0; i<cloud_in->size(); ++i){
    const Point& point = cloud_in->at(i);
    _vertices[3*i] = 2*(point.x-min_x)/x_range-1;
    _vertices[3*i+1] = 2*(point.y-min_y)/y_range-1;
    _vertices[3*i+2] = 2*(point.z-min_z)/z_range-1;
  }

  //[OPENGL] create buffers
  glGenBuffers(1, &_VBO);
  glBindBuffer(GL_ARRAY_BUFFER, _VBO);
  glBufferData(GL_ARRAY_BUFFER, _vertices.size()*sizeof(float), &_vertices[0], GL_STATIC_DRAW);

  glGenVertexArrays(1, &_VAO);
  glBindVertexArray(_VAO);

  //[OPENGL] link vertex attributes
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

}

void Viewer::framebuffer_size_callback(GLFWwindow *window, int width, int height){
  glViewport(0, 0, width, height);
}
