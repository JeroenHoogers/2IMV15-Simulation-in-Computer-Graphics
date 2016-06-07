#pragma once
#if defined(HAVE_CONFIG_H)
#include <GL/glut.h>
#elif defined(_MSC_VER)
// OpenGL Includes
#include <gl/glew.h> //Needs to be included first
#include <gl/GL.h>
#include <gl/GLU.h>
#include <gl/glext.h>
#include <gl/wglew.h>

#include <gl/freeglut.h>

// Link to libraries
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "freeglut.lib")

#elif defined(__APPLE__)
#include <GL/glut.h>
#endif
