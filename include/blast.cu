/**
 * Copyright 2024 Wissem Chiha
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#define BLAST_TORCH (defined(BLAST_TORCH) ? true : false)
#define BLAST_DEBUG (defined(BLAST_DEBUG) ? true : false)

#if defined(__NVCC__)
#include <cuda_runtime.h>
#include <cublas_v2.h>
#endif
#if BLAST_TORCH
#include <torch/torch.h>
#endif
#if BLAST_DEBUG
#include <iostream>
#endif

#include <sciplot/sciplot.hpp>
#include <string>

namespace blast
{ 

template<typename Precision=double>
class Data
{
 
};

template<class Precision=double>
class Trajectory
{
private:

public:
    virtual ~Trajectory()=default;
    virtual Precision getDuration() const = 0;
};

template<class Precision= double,size_t dim>
class CSpace
{   
private:
    Precision* configuration;
public:
    CSpace(){
        err = cudaMalloc(&configuration, dim * sizeof(Precision));
        cudaMemset(configuration, 0, dim * sizeof(Precision));
        if (err!=cudaSuccess)
        {
            std::cerr<<"CUDA memory allocation failed: "<< cudaGetErrorString(err)<<std::endl;
            configuration = nullptr;
        }
    };

    std::vector<Precision> getConfiguration() const {
        std::vector<Precision> hostConfig(dim);
        if (configuration) {
            cudaMemcpy(hostConfig.data(),configuration,dim*sizeof(Precision),cudaMemcpyDeviceToHost);
        } else {
            std::cerr << "configuration memory not allocated on device." << std::endl;
        }
        return hostConfig;
    }

    ~CSpace(){
        cudaFree(configuration);
    }

    #if defined(BLAST_DEBUG)
    
        void printConfiguration() {
            Precision* h_configuration = new Precision[dim];
            cudaMemcpy(h_configuration, configuration, dim*sizeof(Precision),cudaMemcpyDeviceToHost);
            std::cout << "configuration: ";
            for (size_t i = 0; i < dim; ++i) { std::cout << h_configuration[i] << " "; }
            std::cout << std::endl;
            delete[] h_configuration;
        }
        
    #endif

};

template<class Precision=double>
class TaskSpace
{
    
};

template<class Precision=double>
class World
{
private:
    Precision windowWidth;
    Precision windowLength;
    const char* windowTitle;
    const int bufferErrorSize;
    string sceneFilePath;
    int bufferSwap;
public:
    World(){}
    init(){}

};


template <class Precision = double>
class Control
{

};

template<class Precision=double>
class Contact
{


};

template<class Precision=double>
class Collision
{
    virtual Collision(){};
    virtual ~Collision()=default;
};

 
}; // namespace blast




























#include<string>
using namespace std;

/**
 * @brief global c++ structure that handle global variable settings for the 
 *        simulations and main programs
 * @param initialposition   - initial position of the robot 
 * @param windowWidth       - simulation GUI window width size
 * @param windowLength      - simulation GUI window length size
 * @param bufferSwap        - set equal to 1 or 0 
 * @param bufferErrorSize   - the size of the buffer error text returned 
 * @param geomtrySceneNb    - number of geomtries displayed in a scene
 * @param windowTitle       - simulation GUI window title
 * @param simTime           - simulation duration time (seconds)
 * @param viewport          - framebuffer simulation viewport
 * @param modelFilePath     - default path of the robot model xml file   
 * @param sceneFilePath     - default path of the scene model xml file
 * @param modelTxtfile      - default path of the .txt mojoco model file
 * @param figureFilePath    - default path of figures folder
 * @param windowIconPath    - default path of openGL windows icon     
 * 
*/
struct global
{     
    static   double       intialPosition[6]  ;
    static   int          windowWidth        ;
    static   int          windowLength       ;
    static   int          bufferSwap         ;
    static   int          bufferErrorSize    ;
    static   int          geomtryScene       ;
    static   double       simTime            ;
    static   const char*  windowTitle        ;
    static   string       modelFilePath      ;
    static   string       sceneFilePath      ;
    static   const char*  modelTxtfile       ;
    static   const char*  figureFilePath     ;
    static   const char*  windowIconPath     ;
   

};
//global.cpp

#include<string>
using namespace std;

const char* global::windowTitle       ="Mujoco-Simulation"  ; 
string      global::modelFilePath     ="../model/scene.xml" ;
string      global::sceneFilePath     ="../model/scene.xml" ;
const char* global::modelTxtfile      ="../tmp/ur5e.txt"    ;
const char* global::windowIconPath    ="../tmp/icon.png"    ;
const char* global::figureFilePath    = "../tmp"            ;
double      global::simTime           = 0.001                ;
int         global::geomtryScene      = 1000                ;
int         global::bufferErrorSize   = 1000                ;
int         global::bufferSwap        = 1                   ;
int         global::windowLength      = 1000                ;
int         global::windowWidth       = 1200                ;
double      global::intialPosition[6] = {.0,.0,.0,.0,.0,.0} ; 



/**utils.cpp
#include"utils.hpp"
#include<vector>
#include<string>
#include<iostream>
#include<fstream>
 
using namespace std;
 

double point3D::x = .0;
double point3D::y = .0;
double point3D::z = .0;

void Data::array2csv(const std::vector<double>& array, const std::string& filename) {
    std::ofstream file(filename);

    if (file.is_open()) {
        for (size_t i = 0; i < array.size(); ++i) {
            file << array[i];
            if (i < array.size() - 1) {
                file << ",";
            }
        }
        file.close();
        std::cout << "Data written to " << filename << " successfully." << std::endl;
    }
    else{
        std::cerr << "Error: Unable to open file " << filename << std::endl;
    }
} 
//utils.hpp
#ifndef UTILS_HPP
#define UTILS_HPP
#include<vector>
#include<string>
 
 

struct point3D
{
    static double x ;
    static double y ;
    static double z ;
};


struct  Data
{
    static void array2csv(const std::vector<double>& array, const std::string& filename);
};


#include"control.hpp"
#include<iostream>
#include"mujoco/mujoco.h"
 

const char *         control::BodyName    = "wrist_3_link" ;
std::vector<double>  control::BodyPos     = {0.0,0.0,0.0}  ;
double               control::damping        = 0.4           ;
std::vector<double>  control::endEffectorPos = {0.0,0.0,0.0}  ;


void control::dampController(mjModel *m, mjData *d, double damping )
{   
    // set d->ctrl = d->qvel* scl .
    mju_scl(d->ctrl, d->qvel, damping, m->nv);
}


void control::getBodyPose(const mjModel *m, mjData * d, const char* bodyName)
{
    // get the index of the body by name
    int id= mj_name2id(m,mjOBJ_BODY, bodyName);
    // check if the body exists
    if (id >= 0 ){
        int qposadr = m->jnt_qposadr[m->body_jntadr[id]];
        // compute forward dynamics 
        mj_forward(m,d); 
        // set the body pose vector values  
        control::BodyPos[0]=d->xpos[3*qposadr+1];
        control::BodyPos[1]=d->xpos[3*qposadr+2];
        control::BodyPos[2]=d->xpos[3*qposadr+3];
    }else
    {   // body not found 
        std::cout << "body "<< bodyName << "not found !";
    }
}

void control::savePos(const mjModel* m, mjData* d , const char* posFilename){
    
}

void control::forwardKinematics(const mjModel*m, mjData*d){

control::endEffectorPos[0]=cos(d->qpos[0])*sin(d->qpos[1]+d->qpos[2]+d->qpos[3]);
control::endEffectorPos[1]=1;
control::endEffectorPos[2]=1;

}


#ifndef CONTROL_HPP
#define CONTROL_HPP

#include"mujoco/mujoco.h"
#include<vector>
 


struct control
{   
    static const char*         BodyName      ;
    static std::vector<double> BodyPos       ;
    static double              damping       ;
    static std::vector<double> endEffectorPos;
  
  
    static void dampController(mjModel* m, mjData* d, double damping);


    void static setBodyName(const char* BodyName);
   
    static void getBodyPose(const mjModel* m, mjData *d , const char* bodyName);
  
    static void savePos(const mjModel* m, mjData* d , const char* posFilename);

   static void forwardKinematics(const mjModel*m, mjData*d);

/
   static void mpc(const mjModel*m, mjData*d, int64_t H, int64_t M,
                   std::vector<float> Weights,float Ts,
                   std::vector<float> referenceTrajectory);




};









*/





