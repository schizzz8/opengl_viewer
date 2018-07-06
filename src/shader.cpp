#include "shader.h"

using namespace std;

Shader::Shader(const GLchar *vertexPath, const GLchar *fragmentPath){
  string vertexCode;
  string fragmentCode;

  ifstream vShaderFile;
  ifstream fShaderFile;

  vShaderFile.exceptions (ifstream::failbit | ifstream::badbit);
  fShaderFile.exceptions (ifstream::failbit | ifstream::badbit);

  try{
    vShaderFile.open(vertexPath);
    fShaderFile.open(fragmentPath);

    stringstream vShaderStream, fShaderStream;

    vShaderStream << vShaderFile.rdbuf();
    fShaderStream << fShaderFile.rdbuf();

    vShaderFile.close();
    fShaderFile.close();

    vertexCode = vShaderStream.str();
    fragmentCode = fShaderStream.str();

  } catch (ifstream::failure e) {
    cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << endl;
  }

  const char* vShaderCode = vertexCode.c_str();
  const char* fShaderCode = fragmentCode.c_str();

  unsigned int vertex, fragment;
  int success;
  char infoLog[512];

  vertex = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex, 1, &vShaderCode, NULL);
  glCompileShader(vertex);
  glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
  if(!success){
    glGetShaderInfoLog(vertex, 512, NULL, infoLog);
    cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << endl;
  }

  fragment = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment, 1, &fShaderCode, NULL);
  glCompileShader(fragment);
  glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
  if(!success){
    glGetShaderInfoLog(fragment, 512, NULL, infoLog);
    cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << endl;
  }

  this->ID = glCreateProgram();
  glAttachShader(this->ID, vertex);
  glAttachShader(this->ID, fragment);
  glLinkProgram(this->ID);

  glGetProgramiv(this->ID, GL_LINK_STATUS, &success);
  if(!success){
    glGetProgramInfoLog(this->ID, 512, NULL, infoLog);
    cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << endl;
  }

  // delete the shaders as they’re linked into our program now and no longer necessery
  glDeleteShader(vertex);
  glDeleteShader(fragment);
}

void Shader::use(){
  glUseProgram(ID);
}

void Shader::setBool(const string &name, bool value) const {
  glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
}

void Shader::setInt(const std::string &name, int value) const {
  glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
}

void Shader::setFloat(const std::string &name, float value) const {
  glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
}
