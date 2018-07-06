#include "viewer.h"

int main(int argc, char** argv){

  //init viewer
  Viewer viewer;
  viewer.init();

  //load cloud
  viewer.loadCloud(argv[1]);

  //RENDER LOOP
  while(!viewer.windowShouldClose()){

    //process input
    viewer.processInput();

    //rendering commands
    viewer.render();

    //swap buffers and read input
    viewer.update();
  }

  //terminate
  viewer.terminate();
  return 0;
}
