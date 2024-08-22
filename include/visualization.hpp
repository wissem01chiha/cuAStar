/*************************************************************************
	VIZ Engine define visulisation an dredning function and routines 
 ************************************************************************/

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include<iostream>
#include<random>
#include<cmath>
#if defined(ENABLE_CV2) && defined(ENABLE_VTK)
  #error "Only one ENABLE_CV2 and ENABLE_VTK Required"
#elif !defined(ENABLE_CV2) && !defined(ENABLE_VTK)
  #error "ENABLE_CV2 nor ENABLE_VTK is Required."
#endif
#ifdef ENABLE_CV2
  #include<opencv2/opencv.hpp>
  #include<opencv2/core/core.hpp>
  #include<opencv2/highgui/highgui.hpp>
#endif

#ifdef ENABLE_VTK
  #include <vtkActor.h>
  #include <vtkCamera.h>
  #include <vtkConeSource.h>
  #include <vtkNamedColors.h>
  #include <vtkNew.h>
  #include <vtkPolyDataMapper.h>
  #include <vtkProperty.h>
  #include <vtkRenderWindow.h>
  #include <vtkRenderer.h>
#endif






cv::Point2i cv_offset(
    float x, float y,
    int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + image_width/2;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
};

#endif
