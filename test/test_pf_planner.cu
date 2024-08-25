int main(){
  float time=0.0;

  // control input
  Eigen::Vector2f u;
  u<<1.0, 0.1;

  // nosie control input
  Eigen::Vector2f ud;

  // observation z
  std::vector<Eigen::RowVector3f> z;

  // RFID remarks
  Eigen::Matrix<float, 4, 2> RFID;
  RFID<<10.0, 0.0,
        10.0, 10.0,
        0.0,  15.0,
        -5.0, 20.0;

  // dead reckoning
  Eigen::Vector4f xDR;
  xDR<<0.0,0.0,0.0,0.0;

  // ground truth reading
  Eigen::Vector4f xTrue;
  xTrue<<0.0,0.0,0.0,0.0;

  // Estimation
  Eigen::Vector4f xEst;
  xEst<<0.0,0.0,0.0,0.0;

  std::vector<Eigen::Vector4f> hxDR;
  std::vector<Eigen::Vector4f> hxTrue;
  std::vector<Eigen::Vector4f> hxEst;

  Eigen::Matrix4f PEst = Eigen::Matrix4f::Identity();

  // Motional model covariance
  float Q = 0.01;

  // Observation model covariance
  Eigen::Matrix2f  R = Eigen::Matrix2f::Identity();
  R(0,0)=1.0;
  R(1,1)=40.0/180.0 * PI * 40.0/180.0 * PI;

  // Motion model simulation error
  float Qsim = 0.04;

  // Observation model simulation error
  Eigen::Matrix2f Rsim = Eigen::Matrix2f::Identity();
  Rsim(0,0)=1.0 * 1.0;
  Rsim(1,1)=30.0/180.0 * PI * 30.0/180.0 * PI;

  // particle stor
  Eigen::Matrix<float, 4, NP> px = Eigen::Matrix<float, 4, NP>::Zero();

  Eigen::Matrix<float, NP, 1> pw = Eigen::Matrix<float, NP, 1>::Ones() * 1.0/NP;

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> gaussian_d{0,1};
  std::random_device rd2{};
  std::mt19937 gen2{rd2()};
  std::uniform_real_distribution<> uni_d{1.0, 2.0};

  //for visualization
  cv::namedWindow("pf", cv::WINDOW_NORMAL);
  int count = 0;

  while(time <= SIM_TIME){
    time += DT;

    ud(0) = u(0) + gaussian_d(gen) * Rsim(0,0);
    ud(1) = u(1) + gaussian_d(gen) * Rsim(1,1);

    xTrue = motion_model(xTrue, u);
    xDR = motion_model(xDR, ud);

    z.clear();
    for(int i=0; i<RFID.rows(); i++){
      float dx = xTrue(0) - RFID(i, 0);
      float dy = xTrue(1) - RFID(i, 1);
      float d = std::sqrt(dx*dx + dy*dy);
      if (d <= MAX_RANGE){
        float dn = d + gaussian_d(gen) * Qsim;
        Eigen::RowVector3f zi;
        zi<<dn, RFID(i, 0), RFID(i, 1);
        z.push_back(zi);
      }
    }

    pf_localization(px, pw, xEst, PEst, z, u, Rsim, Q, gen, gaussian_d);
    resampling(px, pw, gen2, uni_d);

    // TODO visualization
    hxDR.push_back(xDR);
    hxTrue.push_back(xTrue);
    hxEst.push_back(xEst);

    // visualization
    cv::Mat bg(3500,3500, CV_8UC3, cv::Scalar(255,255,255));
    for(unsigned int j=0; j<hxDR.size(); j++){

      // // green groundtruth
      cv::circle(bg, cv_offset(hxTrue[j].head(2), bg.cols, bg.rows),
                 7, cv::Scalar(0,255,0), -1);

      // blue estimation
      cv::circle(bg, cv_offset(hxEst[j].head(2), bg.cols, bg.rows),
                 10, cv::Scalar(255,0,0), 5);

      // black dead reckoning
      cv::circle(bg, cv_offset(hxDR[j].head(2), bg.cols, bg.rows),
                 7, cv::Scalar(0, 0, 0), -1);
    }


    for(int j=0; j<px.cols(); j++){
      cv::circle(bg, cv_offset(px.col(j).head(2), bg.cols, bg.rows),
                 3, cv::Scalar(0, 0, 255), -1);
    }

    for(int i=0; i<RFID.rows(); i++){
      cv::circle(bg, cv_offset(RFID.row(i), bg.cols, bg.rows),
                 20, cv::Scalar(127, 0, 255), -1);
    }
    for(unsigned int i=0; i<z.size(); i++){
      cv::line(
        bg,
        cv_offset(z[i].tail(2), bg.cols, bg.rows),
        cv_offset(hxEst.back().head(2), bg.cols, bg.rows),
        cv::Scalar(0, 0, 0),
        5);
    }

    ellipse_drawing(bg, PEst.block(0,0,2,2), xEst.head(2));

    cv::imshow("pf", bg);
    cv::waitKey(5);

    // std::string int_count = std::to_string(count);
    // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);

    count++;
  }
}
