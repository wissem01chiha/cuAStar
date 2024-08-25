/**
 * @file 
 */
#include<iostream>
#include<vector>
#include<random>
#include<cmath>


#define SIM_TIME 50.0
#define DT 0.1
#define PI 3.141592653
#define MAX_RANGE 20.0
#define NP 100
#define NTh NP/2

namespace SimulationParams {
    constexpr float SIM_TIME = 50.0f;
    constexpr float DT = 0.1f;
    constexpr float PI = 3.141592653f;
    constexpr float MAX_RANGE = 20.0f;
    constexpr int NP = 100;
    constexpr int NTh = NP / 2;
}


// x_{t+1} = F@x_{t}+B@u_t
Eigen::Vector4f motion_model(Eigen::Vector4f x, Eigen::Vector2f u){
  Eigen::Matrix4f F_;
  F_<<1.0,   0,   0,   0,
        0, 1.0,   0,   0,
        0,   0, 1.0,   0,
        0,   0,   0, 1.0;

  Eigen::Matrix<float, 4, 2> B_;
  B_<< DT * std::cos(x(2,0)),  0,
       DT * std::sin(x(2,0)),  0,
                        0.0,  DT,
                        1.0,  0.0;

  return F_ * x + B_ * u;
};


//observation mode H
Eigen::Vector2f observation_model(Eigen::Vector4f x){
  Eigen::Matrix<float, 2, 4> H_;
  H_<< 1, 0, 0, 0,
       0, 1, 0, 0;
  return H_ * x;
};


// TODO gaussian likelihood
float gauss_likelihood(float x, float sigma){
  float p = 1.0 / std::sqrt(2.0 * PI * sigma * sigma) * \
      std::exp(-x * x / (2 * sigma * sigma));
  return p;
};

Eigen::Matrix4f calc_covariance(
    Eigen::Vector4f xEst,
    Eigen::Matrix<float, 4, NP> px,
    Eigen::Matrix<float, NP, 1> pw){

  Eigen::Matrix4f PEst_ = Eigen::Matrix4f::Zero();
  for(int i=0; i<px.cols(); i++){
      Eigen::Vector4f dx = px.col(i) - xEst;
      PEst_ += pw(i) * dx * dx.transpose();
  }

  return PEst_;
};

void pf_localization(
    Eigen::Matrix<float, 4, NP>& px, Eigen::Matrix<float, NP, 1>& pw,
    Eigen::Vector4f& xEst, Eigen::Matrix4f& PEst,
    std::vector<Eigen::RowVector3f> z, Eigen::Vector2f u,
    Eigen::Matrix2f Rsim, float Q,
    std::mt19937 gen,  std::normal_distribution<> gaussian_d
    ){

  for(int ip=0; ip<NP; ip++){
    Eigen::Vector4f x = px.col(ip);
    float w = pw(ip);

    Eigen::Vector2f ud;

    ud(0) = u(0) + gaussian_d(gen) * Rsim(0,0);
    ud(1) = u(1) + gaussian_d(gen) * Rsim(1,1);

    x = motion_model(x, ud);

    for(unsigned int i=0; i<z.size(); i++){
        Eigen::RowVector3f item = z[i];
        float dx = x(0) - item(1);
        float dy = x(1) - item(2);
        float prez = std::sqrt(dx*dx + dy*dy);
        float dz = prez - item(0);
        w = w * gauss_likelihood(dz, std::sqrt(Q));
    }
    px.col(ip) = x;
    pw(ip) = w;
  }

  pw = pw / pw.sum();

  xEst = px * pw;
  PEst = calc_covariance(xEst, px, pw);

};

Eigen::Matrix<float, NP, 1> cumsum(Eigen::Matrix<float, NP, 1> pw){
  Eigen::Matrix<float, NP, 1> cum;
  cum(0) = pw(0);
  for(int i=1; i<pw.rows(); i++){
    cum(i) = cum(i-1) + pw(i);
  }
  return cum;
}

void resampling(
    Eigen::Matrix<float, 4, NP>& px,
    Eigen::Matrix<float, NP, 1>& pw,
    std::mt19937 gen,
    std::uniform_real_distribution<> uni_d){

  float Neff = 1.0 / (pw.transpose() * pw);
  if (Neff < NTh){
    Eigen::Matrix<float, NP, 1> wcum = cumsum(pw);
    Eigen::Matrix<float, NP, 1> base = cumsum(pw * 0.0 +  Eigen::Matrix<float, NP, 1>::Ones()*1.0/NP) - Eigen::Matrix<float, NP, 1>::Ones()*1.0/NP;
    Eigen::Matrix<float, NP, 1> resampleid;
    Eigen::Matrix<float, 4, NP> output;
    for(int j=0; j<pw.rows(); j++){
      resampleid(j) = base(j) + uni_d(gen)/NP;
    }

    int ind = 0;

    for(int i=0; i<NP; i++){
        while(resampleid(i) > wcum(ind) && ind<NP-1){
          ind += 1;
        }
        output.col(i) = px.col(ind);
    }

    px = output;
    pw = Eigen::Matrix<float, NP, 1>::Ones()*1.0/NP;
  }
};

cv::Point2i cv_offset(
    Eigen::Vector2f e_p, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(e_p(0) * 100) + image_width/2;
  output.y = image_height - int(e_p(1) * 100) - image_height/3;
  return output;
};

void ellipse_drawing(
    cv::Mat bg_img, Eigen::Matrix2f pest, Eigen::Vector2f center,
    cv::Scalar ellipse_color={0, 0, 255}){
  Eigen::EigenSolver<Eigen::Matrix2f> ces(pest);
  Eigen::Matrix2f e_value = ces.pseudoEigenvalueMatrix();
  Eigen::Matrix2f e_vector = ces.pseudoEigenvectors();

  double angle = std::atan2(e_vector(0, 1), e_vector(0, 0));
  cv::ellipse(
    bg_img,
    cv_offset(center, bg_img.cols, bg_img.rows),
    cv::Size(e_value(0,0)*1000, e_value(1,1)*1000),
    angle / PI * 180,
    0,
    360,
    ellipse_color,
    2,
    4);
};

