#include "viewer.h"

Viewer::Viewer(){
  _w = 800;
  _h = 600;

  _model = glm::mat4(1.0f);
  _view = glm::translate(_view, glm::vec3(0.0f, 0.0f, -5.0f));
  _projection = glm::perspective(glm::radians(60.0f), _w/(float)_h, 0.001f, 100.0f);

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

  //[OPENGL] create shaders
  _shader = Shader("../shaders/point.vs", "../shaders/point.fs");

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
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //2) set shaders to use
  _shader.use();

  //3) set shaders uniform
  int modelLoc = glGetUniformLocation(_shader.ID, "model");
  glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(_model));
  int viewLoc = glGetUniformLocation(_shader.ID, "view");
  glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(_view));
  int projLoc = glGetUniformLocation(_shader.ID, "projection");
  glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(_projection));

  //3) bind VAO
  glBindVertexArray(_VAO);

  //4) draw
  glDrawArrays(GL_POINTS,0,_num_vertices);


}

void Viewer::update(){
  glfwSwapBuffers(_window);
  glfwPollEvents();
}

void Viewer::terminate(){
  //[GLFW] terminate
  glfwTerminate();
}

void Viewer::loadCloud(char *filename){
  //[PCL] load cloud
  PointCloud::Ptr cloud_in (new PointCloud);
  if (pcl::io::loadPCDFile<Point> (filename, *cloud_in) == -1) {
    PCL_ERROR ("Couldn't read file!\n");
    return;
  }

  _num_vertices = cloud_in->size();

  _vertices.resize(_num_vertices*6);
  for(size_t i=0; i<_num_vertices; ++i){
    const Point& point = cloud_in->at(i);
    _vertices[6*i] = point.x;
    _vertices[6*i+1] = point.y;
    _vertices[6*i+2] = point.z;
    _vertices[6*i+3] = point.r/255.0f;
    _vertices[6*i+4] = point.g/255.0f;
    _vertices[6*i+5] = point.b/255.0f;
  }

//  std::cerr << "Vertices:" << std::endl;
//  for(size_t i=0; i<cloud_in->size(); ++i){
//    std::cerr << _vertices[6*i] << " " << _vertices[6*i+1] << " " << _vertices[6*i+2] << " ";
//    std::cerr << _vertices[6*i+3] << " " << _vertices[6*i+4] << " " << _vertices[6*i+5] << std::endl;
//  }
}

void Viewer::generateVertices(){
  _vertices.resize(18);
  _vertices[0]=-0.5f;
  _vertices[1]=-0.5f;
  _vertices[2]=0.0f;
  _vertices[3]=0.f;
  _vertices[4]=0.f;
  _vertices[5]=0.f;
  _vertices[6]=0.5f;
  _vertices[7]=-0.5f;
  _vertices[8]=0.0f;
  _vertices[9]=0.f;
  _vertices[10]=0.f;
  _vertices[11]=0.f;
  _vertices[12]=0.0f;
  _vertices[13]=0.5f;
  _vertices[14]=0.0f;
  _vertices[15]=0.f;
  _vertices[16]=0.f;
  _vertices[17]=0.f;
}

void Viewer::bindBuffers(){
  if(_vertices.empty())
    return;

  //[OPENGL] create buffers
  glGenBuffers(1, &_VBO);
  glBindBuffer(GL_ARRAY_BUFFER, _VBO);
  glBufferData(GL_ARRAY_BUFFER, _vertices.size()*sizeof(float), &_vertices[0], GL_STATIC_DRAW);

  glGenVertexArrays(1, &_VAO);
  glBindVertexArray(_VAO);

  //[OPENGL] link vertex attributes
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3* sizeof(float)));
  glEnableVertexAttribArray(1);
}

void Viewer::framebuffer_size_callback(GLFWwindow *window, int width, int height){
  glViewport(0, 0, width, height);
}
