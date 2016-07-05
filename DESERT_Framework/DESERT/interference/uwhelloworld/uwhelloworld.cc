#include "uwhelloworld.h"

#include <iostream>

/**
 * Class that represents the binding with the tcl configuration script 
 */
static class Helloworld_Class : public TclClass {
public:
  /**
   * Constructor of the class
   */
  Helloworld_Class() : TclClass("Module/UW/HELLOWORLD") {}
  /**
   * Creates the TCL object needed for the tcl language interpretation
   * @return Pointer to an TclObject
   */
  TclObject* create(int, const char*const*) {
    return (new uwhelloworld);
  }
} class_helloworld;


uwhelloworld::uwhelloworld() {
  bind("debug_", &debug_);
}

int uwhelloworld::command(int argc, const char *const *argv) {
  if (argc == 2) {
    if (strcmp(argv[1], "Hello") == 0) {
      std::cout << "Hello world!" << std::endl;
      return TCL_OK;
    }
  }
  return PlugIn::command(argc, argv);
}
