#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

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
#define ENABLE_VTK 1
#ifdef ENABLE_VTK
  #include <vtkActor.h>
  #include <vtkNamedColors.h>
  #include <vtkNew.h>
  #include <vtkPLYReader.h>
  #include <vtkSphereSource.h>
  #include <vtkPolyData.h>
  #include <vtkPolyDataMapper.h>
  #include <vtkProperty.h>
  #include <vtkRenderWindow.h>
  #include <vtkRenderWindowInteractor.h>
  #include <vtkRenderer.h>
  #include <vtkUnsignedCharArray.h>
  #include <vtkPointData.h>
  #include <vtkAppendPolyData.h>
  #include <vtkSmartPointer.h>
#endif

#include "utils.hpp"
#include "common.hpp"

/**
 * @brief Display a node as a sphere with a specified radius and color.
 * @param node Pointer to the Node3d object to visualize.
 * @param radius Radius of the sphere (default is 0.01).
 * @param color Array of size 3 containing RGB values for the color.
 * @note this function has no cv2 implementation.
 */
void vizNode3d(const internal::Node3d* node, double radius = 0.01, double color[3]) {
  #ifdef ENABLE_VTK
    vtkNew<vtkNamedColors> colors;
    vtkNew<vtkSphereSource> nodeSource;
    nodeSource->SetCenter(node->x, node->y, node->z);
    nodeSource->SetRadius(radius);
    nodeSource->SetThetaResolution(20);
    nodeSource->SetPhiResolution(20);
    nodeSource->Update();

    vtkNew<vtkPolyDataMapper> nodeSourceMapper;
    nodeSourceMapper->SetInputConnection(nodeSource->GetOutputPort());
    nodeSourceMapper->ScalarVisibilityOn();  

    vtkNew<vtkActor> nodeSourceActor;
    nodeSourceActor->SetMapper(additionalSphereMapper);
    nodeSourceActor->GetProperty()->SetColor(color[0] ,color[1],color[2]);  
    nodeSourceActor->GetProperty()->SetOpacity(1.0);

    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkRenderWindow> renderWindow;
    renderWindow->AddRenderer(renderer);
    vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderer->AddActor(nodeSourceActor);
    renderer->SetBackground(colors->GetColor3d("White").GetData());
    renderWindow->Render();
    renderWindowInteractor->Start();
  #endif
};

/**
 * @brief Display a 2d node as a circle with a specified radius and color
 */
void vizNode2d(const internal::Node2d* node, double radius = 0.01, double color[3]){
  #ifdef ENABLE_VTK

  #elif defined(ENABLE_CV2)
    cv::Mat img = cv::Mat::zeros(400, 400, CV_8UC3);
    cv::Point center(node->x, node->y);
    cv::Scalar color(color[0], color[1],color[2]);
    cv::circle(img, center, radius, color, -1);
    for (int i = 0; i < radius; ++i) {
        double factor = 1.0 - (double)i / (double)radius;
        cv::circle(img, center, radius - i, color * factor, 1);
    }
    cv::imshow("2D Node2d", img);
    cv::waitKey(0);
  #endif
};

/**
 * @brief Visualizes the entire 3D point cloud from a specified .ply file.
 * This function renders the point cloud contained in the given .ply file, 
 * preserving the original RGBA colors.
 * @note This function does not have an implementation using OpenCV (cv2).
 * It relies on external visualization libraries, and the point cloud data
 * must include RGBA color information.
 * @param inputFilePath The path to the input .ply file containing the 3D point 
 * cloud data.
 * @param pointsRadius The radius for all points in the visualization. 
 * @note Use a smaller radius for large point clouds to 
 * ensure clarity and performance. The default value is 0.01.
 */
void vizWorld3d(const std::string inputFilePath,const double pointsRadius=0.01){
  #ifdef ENABLE_VTK
    vtkNew<vtkNamedColors> colors;
    vtkNew<vtkPLYReader> reader;
    reader->SetFileName(inputFilePath.c_str());
    reader->Update();                           

    vtkPolyData* polyData = reader->GetOutput();
    vtkPointData* pointData = polyData->GetPointData();

    vtkUnsignedCharArray* colorsArray=
      vtkUnsignedCharArray::SafeDownCast(pointData->GetArray("RGBA"));
    if (!colorsArray){
        std::cerr << 
        "Error:The PLY file does not contain an array named 'RGBA'!"<<std::endl;
    }
    vtkIdType numPoints = polyData->GetNumberOfPoints();
    std::cout << "Number of points: " << numPoints << std::endl;

    vtkNew<vtkAppendPolyData> appendFilter;

    for (vtkIdType i = 0; i < numPoints; ++i){
      double p[3];
      polyData->GetPoint(i, p);
      unsigned char* color = colorsArray->GetPointer(4 * i);
      unsigned char rgba[4] = {
        color[0],  
        color[1],  
        color[2],  
        color[3]   
      };

    vtkNew<vtkSphereSource> sphereSource;
    sphereSource->SetCenter(p);
    sphereSource->SetRadius(pointsRadius); / 
    sphereSource->SetThetaResolution(8);
    sphereSource->SetPhiResolution(8);
    sphereSource->Update();

    vtkIdType numSpherePoints = sphereSource->GetOutput()->GetNumberOfPoints();

    vtkNew<vtkUnsignedCharArray> colorArray;
    colorArray->SetNumberOfComponents(4); 
    colorArray->SetName("Colors");
    for (vtkIdType j = 0; j < numSpherePoints; ++j){
        colorArray->InsertNextTypedTuple(rgba);
    }
    sphereSource->GetOutput()->GetPointData()->SetScalars(colorArray);
    appendFilter->AddInputConnection(sphereSource->GetOutputPort());
    };

    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkRenderWindow> renderWindow;
    renderWindow->AddRenderer(renderer);
    vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
    renderWindowInteractor->SetRenderWindow(renderWindow);
 
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(appendFilter->GetOutputPort());
    mapper->ScalarVisibilityOn();  
    mapper->SetScalarModeToUsePointData(); 
    mapper->SelectColorArray("Colors");

    vtkNew<vtkActor> pointCloudActor;
    pointCloudActor->SetMapper(mapper);

    renderer->AddActor(pointCloudActor);
    renderer->SetBackground(colors->GetColor3d("White").GetData());
    renderWindow->Render();
    renderWindowInteractor->Start();
  #endif
};

/**
 * @brief display the 2d trajectory data using a 2d trajectory file
 * @note 2d trajectory files are .ply or .csv for both vtk or opencv
 * in .ply if 3 compenant is prsent "z" it is igonred by default.
 */
void viz2dTrajectory(){
  #ifdef ENABLE_VTK

  #endif
};

/**
 * @brief display the 2d trajectory data using a 3d trajectory file 
 * a 3d trajectory file is a non rgba point cloud file stored in .ply 
 * format  
 * @note this function do not have cv2 implemantion
 */
void viz3dTrajectory(const std::string inputFilePath){
  #ifdef ENABLE_VTK

  #endif
};
#endif