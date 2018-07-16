#include "viewer.h"

Viewer::Viewer(){
  _w = 800;
  _h = 600;

  _model = glm::mat4(1.0f);
  _view = glm::translate(_view, glm::vec3(0.0f, 0.0f, -5.0f));
  _projection = glm::perspective(glm::radians(45.0f), _w/(float)_h, 0.001f, 100.0f);

  _box_vertices.clear();
  _cloud_vertices.clear();

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

  for(size_t i=0; i<cloud_in->size(); ++i){
    const Point& point = cloud_in->at(i);
    insertVertex(_cloud_vertices,Eigen::Vector3f(point.x,point.y,point.z),Eigen::Vector3f(point.r/255.0f,point.g/255.0f,point.b/255.0f));
  }
  _num_cloud_vertices = _cloud_vertices.size()/6.0f;

  //  std::cerr << "Cloud vertices:" << std::endl;
  //  for(size_t i=0; i<cloud_in->size(); ++i){
  //    std::cerr << _vertices[6*i] << " " << _vertices[6*i+1] << " " << _vertices[6*i+2] << " ";
  //    std::cerr << _vertices[6*i+3] << " " << _vertices[6*i+4] << " " << _vertices[6*i+5] << std::endl;
  //  }
}

//const GLfloat data[48] = {
//  // Loop 1: XY Z (min)
//  aabb.mMin.x, aabb.mMin.y, aabb.mMin.z,
//  aabb.mMax.x, aabb.mMin.y, aabb.mMin.z,
//  aabb.mMax.x, aabb.mMax.y, aabb.mMin.z,
//  aabb.mMin.x, aabb.mMax.y, aabb.mMin.z,

//  // Loop 2: XY Z (max)
//  aabb.mMin.x, aabb.mMin.y, aabb.mMax.z,
//  aabb.mMax.x, aabb.mMin.y, aabb.mMax.z,
//  aabb.mMax.x, aabb.mMax.y, aabb.mMax.z,
//  aabb.mMin.x, aabb.mMax.y, aabb.mMax.z,

//  // Lists:
//  // 1
//  aabb.mMin.x, aabb.mMin.y, aabb.mMin.z,
//  aabb.mMin.x, aabb.mMin.y, aabb.mMax.z,
//  // 2
//  aabb.mMax.x, aabb.mMin.y, aabb.mMin.z,
//  aabb.mMax.x, aabb.mMin.y, aabb.mMax.z,
//  // 3
//  aabb.mMax.x, aabb.mMax.y, aabb.mMin.z,
//  aabb.mMax.x, aabb.mMax.y, aabb.mMax.z,
//  // 4
//  aabb.mMin.x, aabb.mMax.y, aabb.mMin.z,
//  aabb.mMin.x, aabb.mMax.y, aabb.mMax.z,
//};

void Viewer::loadBox(){
  Eigen::Vector3f min(1.7921,-0.2782,0.0);
  Eigen::Vector3f max(2.1521,0.1994,0.9915);

  insertVertex(_box_vertices,min.x(),min.y(),min.z(),0.0,0.0,1.0);
  insertVertex(_box_vertices,max.x(),min.y(),min.z(),0.0,0.0,1.0);
  insertVertex(_box_vertices,max.x(),max.y(),min.z(),0.0,0.0,1.0);
  insertVertex(_box_vertices,min.x(),max.y(),min.z(),0.0,0.0,1.0);

  _num_box_vertices = _box_vertices.size()/6.0f;
  std::cerr << "Box vertices:" << std::endl;
  for(size_t i=0; i<_num_box_vertices; ++i){
    std::cerr << _box_vertices[6*i] << " " << _box_vertices[6*i+1] << " " << _box_vertices[6*i+2] << " ";
    std::cerr << _box_vertices[6*i+3] << " " << _box_vertices[6*i+4] << " " << _box_vertices[6*i+5] << std::endl;
  }
}

void Viewer::generateVertices(){
  _cloud_vertices.resize(18);
  _cloud_vertices[0]=-0.5f;
  _cloud_vertices[1]=-0.5f;
  _cloud_vertices[2]=0.0f;
  _cloud_vertices[3]=0.f;
  _cloud_vertices[4]=0.f;
  _cloud_vertices[5]=0.f;
  _cloud_vertices[6]=0.5f;
  _cloud_vertices[7]=-0.5f;
  _cloud_vertices[8]=0.0f;
  _cloud_vertices[9]=0.f;
  _cloud_vertices[10]=0.f;
  _cloud_vertices[11]=0.f;
  _cloud_vertices[12]=0.0f;
  _cloud_vertices[13]=0.5f;
  _cloud_vertices[14]=0.0f;
  _cloud_vertices[15]=0.f;
  _cloud_vertices[16]=0.f;
  _cloud_vertices[17]=0.f;
}

void Viewer::bindBuffers(){
  //  if(_cloud_vertices.empty() || _box_vertices.empty())
  //    return;

  //[OPENGL] create cloud buffers
  //  glGenBuffers(1, &_cVBO);
  //  glBindBuffer(GL_ARRAY_BUFFER, _cVBO);
  //  glBufferData(GL_ARRAY_BUFFER, _cloud_vertices.size()*sizeof(float), &_cloud_vertices[0], GL_STATIC_DRAW);

  //  glGenVertexArrays(1, &_cVAO);
  //  glBindVertexArray(_cVAO);

  //[OPENGL] link cloud vertex attributes
  //  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  //  glEnableVertexAttribArray(0);
  //  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3* sizeof(float)));
  //  glEnableVertexAttribArray(1);

  //[OPENGL] create box buffers
  glGenBuffers(1, &_bVBO);
  glBindBuffer(GL_ARRAY_BUFFER, _bVBO);
  glBufferData(GL_ARRAY_BUFFER, _box_vertices.size()*sizeof(float), &_box_vertices[0], GL_STATIC_DRAW);

  glGenVertexArrays(1, &_bVAO);
  glBindVertexArray(_bVAO);

  //[OPENGL] link box vertex attributes
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3* sizeof(float)));
  glEnableVertexAttribArray(1);

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

  //3) bind cloud VAO
  //  glBindVertexArray(_cVAO);

  //4) draw cloud
  //  glDrawArrays(GL_POINTS,0,_num_cloud_vertices);

  //3) bind box VAO
  glBindVertexArray(_bVAO);

  //4) draw box
  glDrawArrays(GL_LINE_LOOP,0,_num_box_vertices);

}

void Viewer::framebuffer_size_callback(GLFWwindow *window, int width, int height){
  glViewport(0, 0, width, height);
}

void Viewer::insertVertex(std::vector<float> &vertices,
                          const Eigen::Vector3f &vertex,
                          const Eigen::Vector3f &color){
  vertices.push_back(vertex.x());
  vertices.push_back(vertex.y());
  vertices.push_back(vertex.z());
  vertices.push_back(color.x());
  vertices.push_back(color.y());
  vertices.push_back(color.z());
}

void Viewer::insertVertex(std::vector<float> &vertices, float x, float y, float z, float r, float g, float b){
  vertices.push_back(x);
  vertices.push_back(y);
  vertices.push_back(z);
  vertices.push_back(r);
  vertices.push_back(g);
  vertices.push_back(b);
}
