/*************************************************************************
	VIZ Engine define visulisation an dredning function and routines 
 ************************************************************************/
#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include<iostream>
#include<random>
#include<cmath>
#if defined(ENABLE_CV2) && defined(ENABLE_VTK)
  #error "Only one ENABLE_CV2 and ENABLE_VTK Required"
#elif !defined(ENABLE_CV2) && !defined(ENABLE_VTK)
  #warning "ENABLE_CV2 nor ENABLE_VTK is Required."
#endif
#ifdef ENABLE_CV2
  #include<opencv2/opencv.hpp>
  #include<opencv2/core/core.hpp>
  #include<opencv2/highgui/highgui.hpp>
#endif

#ifdef ENABLE_VTK
  #include <vtkActor.h>
  #include <vtkNamedColors.h>
  #include <vtkNew.h>
  #include <vtkPLYReader.h>
  #include <vtkPolyDataMapper.h>
  #include <vtkProperty.h>
  #include <vtkRenderWindow.h>
  #include <vtkRenderWindowInteractor.h>
  #include <vtkRenderer.h>
#endif

#include "utils.hpp"

void load_map(){
  #ifdef ENABLE_VTK

  #endif
};

void 











 

#endif
