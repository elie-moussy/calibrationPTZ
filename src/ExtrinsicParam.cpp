/***************************************
  * Copyright (C) LAAS-CNRS
  * Author : Elie MOUSSY
***************************************/

#include "calibrationPTZ/ExtrinsicParam.hh"
#include <fstream>

#define PI 3.14159265

//\fn template<typename matrix> bool fprintMatrix(matrix m, std::string name);
///\brief Print an Eigen matrix in a file.
///\param m The Eigen matrix.
///\param name The file name.
template<typename matrix>
bool fprintMatrix(matrix m, std::string name)  
{  
  ofstream stream(name.c_str());
  if (stream)
    {
      int i,j;
      for(i=0; i< m.rows(); ++i)  
	{  
	  for(j=0; j< m.cols(); ++j)  
	    {  
	      stream << m(i,j) << " ";   
	    }  
	  stream << std::endl;  
	}
      stream.close();
      return true;
    }
  return false;
}

//\fn template<typename matrix> bool freadMatrix(matrix &m, std::string name);
///\brief Read an Eigen matrix from a file.
///\param m The Eigen matrix.
///\param name The file name.
template<typename matrix>
bool freadMatrix(matrix &m, std::string name)  
{  
  ifstream stream(name.c_str());
  if (stream)
    {
      int i,j;
      for(i=0; i< m.rows(); ++i)  
	{  
	  for(j=0; j< m.cols(); ++j)  
	    {  
	      stream >> m(i,j);
	      char c;
	      stream.get(c);
	    }
	}
      stream.close();
      return true;
    }
  return false;
}

//\fn bool fprintPT(double pan, double tilt, std::string name);
///\brief Print the values of pan and tilt in a file.
///\param pan The value of pan.
///\param tilt The value of tilt.
///\param name The file name.
bool fprintPT(double pan, double tilt, std::string name)
{
  ofstream stream(name.c_str());
  if (stream)
    {
      stream << pan << " " << tilt;
      stream.close();
      return true;
    }
  return false;
}

//\fn bool freadPT(double &pan, double &tilt, std::string name);
///\brief Read the values of pan and tilt from a file.
///\param pan The value of pan.
///\param tilt The value of tilt.
///\param name The file name.
bool freadPT(double &pan, double &tilt, std::string name)
{
  ifstream stream(name.c_str());
  if (stream)
    {
      stream >> pan;
      char c;
      stream.get(c);
      stream >> tilt;
      stream.close();
      return true;
    }
  return false;
}

//\fn ExtrinsicParam::ExtrinsicParam(int cam);
///\brief The constructor of the class "ExtrinsicParam".
///\param cam Indicates the camera number (1 or 2).
ExtrinsicParam::ExtrinsicParam(int cam):
  pan(0.),
  tilt(0.),
  rotation_computed(false)
{
  bool readingFile = true;
  this->cam = cam;
  dist.resize(5);
  K.setZero();
  rotation.setZero();
  translation.setZero();
  if (cam == 2)
    {
      K(0,0) = 800;
      K(1,1) = 600;
      K(2,2) = 1;
      K(0,2) = 399.92;
      K(1,2) = 234.501;
      dist << -0.113323,
	-0.023496,
	-0.013891,
	-0.003838,
	0.267853;
      if(!freadMatrix(rotation, "rotation_cam2.txt"))
	{
	  std::cout << "Error reading file rotation_cam2.txt" << std::endl;
	  readingFile = false;
	}
      if(!freadMatrix(translation, "translation_cam2.txt"))
	{
	  std::cout << "Error reading file translation_cam2.txt" << std::endl;
	  readingFile = false;
	}
      if(!freadPT(pan,tilt, "PT_cam2.txt"))
	{
	  std::cout << "Error reading file PT_cam2.txt" << std::endl;
	  readingFile = false;
	}
    } else if (cam == 1)
    {
      K(0,0) = 800;
      K(1,1) = 600;
      K(2,2) = 1;
      K(0,2) = 322.114;
      K(1,2) = 217.173;
      dist << -0.133553,
	0.078593,
	0.001123,
	0.001457,
	-0.043746;
      if(!freadMatrix(rotation, "rotation_cam1.txt"))
	{
	  std::cout << "Error reading file rotation_cam1.txt" << std::endl;
	  readingFile = false;
	}
      if(!freadMatrix(translation, "translation_cam1.txt"))
	{
	  std::cout << "Error reading file translation_cam1.txt" << std::endl;
	  readingFile = false;
	}
      if(!freadPT(pan,tilt, "PT_cam1.txt"))
	{
	  std::cout << "Error reading file PT_cam1.txt" << std::endl;
	  readingFile = false;
	}
    }
  initial_rotation.noalias() = rotation;
  initial_translation.noalias() = translation;
  rotation_computed = readingFile;
  std::cout << "rotation=\n" << rotation << std::endl;
  std::cout << "translation=\n" << translation << std::endl;
  std::cout << "pan=" << pan << "\ttilt=" << tilt << std::endl;
}

//\fn ExtrinsicParam::~ExtrinsicParam();
///\brief The destructor of the class "ExtrinsicParam".
ExtrinsicParam::~ExtrinsicParam()
{
  //delete calib;
}

//\fn Eigen::Vector3d ExtrinsicParam::getTranslationVector();
///\brief A getter function of the translation vector.
///\return The 3D vector of translation.
Eigen::Vector3d ExtrinsicParam::getTranslationVector()
{
  return translation;
}

//\fn Eigen::Matrix3d ExtrinsicParam::getRotationMatrix();
///\brief A getter function of the rotation matrix.
///\return The 3x3 matrix of rotation.
Eigen::Matrix3d ExtrinsicParam::getRotationMatrix()
{
  if (rotation_computed)
    return rotation;

  Eigen::Matrix3d zero;
  zero.fill(0.);
  return zero;
}

//\fn Eigen::Matrix3d ExtrinsicParam::getCameraMatrix();
///\brief A getter function of the camera matrix (the intrinsic parameters).
///\return The a 3x3 matrix of the camera calibration.
Eigen::Matrix3d ExtrinsicParam::getCameraMatrix()
{
  return K;
}

//\fn void ExtrinsicParam::computeRtMatrix(double pan, double tilt, cv::Mat image);
///\brief This function computes the rotation matrix and the translation vector of the extrinsic parameters.
///\param pan Value of the camera panoramique when the image has been taken.
///\param tilt Value of the camera tilt when the image has been taken.
///\param image The image from which the rotation matrix and the translation vector should be computed.
void ExtrinsicParam::computeRtMatrix(double pan, double tilt, cv::Mat image)
{
  // Save the value of the pan and the tilt in order to modify the extrinsic parameters when the camera position change
  this->pan = pan;
  this->tilt = tilt;
  cv::Mat img = image.clone();
  cv::Mat initImg = image.clone();
  int cnt = 0;

  double fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);
  double u, v, x, y;
  double dist_to_point;

  Eigen::MatrixXd Pr, Pi;
  Pr.resize(4,6);
  Pr.setZero();
  Pi.resize(3,6);
  Pi.setZero();

  pp.cnt = 0;

  // Create a window of the scene
  cv::namedWindow("Extrinsic Image",CV_WINDOW_AUTOSIZE);
  cvSetMouseCallback("Extrinsic Image",On_Mouse,0);
  cv::waitKey(10);
  cv::imshow("Extrinsic Image", img);
  cv::waitKey(10);

  // We need 6 points to compute the translation vector and the rotation matrix
  while (pp.cnt <= 6)
    {
      if (cnt > pp.cnt) // Case where we do a right click in order to remove the last point inserted
	{
	  switch (cnt)
	    {
	    case 1:
	      {
		img = image.clone();
		cnt = pp.cnt;
	      }
	      break;
	      
	    case 2:
	      {
		img = image.clone();
		cv::circle(img,pp.p1[0],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,0)",pp.p1[0],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    case 3:
	      {
		img = image.clone();
		cv::circle(img,pp.p1[0],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,0)",pp.p1[0],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[1],3,cv::Scalar(255,0,0));
		cv::putText(img,"(1.5,0,0)",pp.p1[1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    case 4:
	      {
		img = image.clone();
		cv::circle(img,pp.p1[0],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,0)",pp.p1[0],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[1],3,cv::Scalar(255,0,0));
		cv::putText(img,"(1.5,0,0)",pp.p1[1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[2],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,1.5,0)",pp.p1[2],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    case 5:
	      {
		img = image.clone();
		cv::circle(img,pp.p1[0],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,0)",pp.p1[0],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[1],3,cv::Scalar(255,0,0));
		cv::putText(img,"(1.5,0,0)",pp.p1[1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[2],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,1.5,0)",pp.p1[2],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[3],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,1)",pp.p1[3],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    case 6:
	      {
		img = image.clone();
		cv::circle(img,pp.p1[0],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,0)",pp.p1[0],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[1],3,cv::Scalar(255,0,0));
		cv::putText(img,"(1.5,0,0)",pp.p1[1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[2],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,1.5,0)",pp.p1[2],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[3],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,1)",pp.p1[3],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[4],3,cv::Scalar(255,0,0));
		cv::putText(img,"(1.5,0,1)",pp.p1[4],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    default:
	      break;
	    }
	  cv::imshow("Extrinsic Image", img);
	  cv::waitKey(10);
	}
      if (pp.clicked) // Case where we do a left click in order to insert a point
	{
	  cv::circle(img,pp.p1[pp.cnt-1],3,cv::Scalar(255,0,0));
	  
	  if (pp.cnt == 1) // First point to insert (0,0,0) (on the mocap basis)
	    {
	      cv::putText(img,"(0,0,0)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      // Get the image coordinates
	      u = pp.p1[0].x;
	      v = pp.p1[0].y;
	      Eigen::Vector3d undist;
	      undist(0) = u;
	      undist(1) = v;
	      undist(2) = 1.;
	      
	      // Get the distance between the camera and the 3d point (the scale s)
	      if (cam == 1)
		{
		  dist_to_point = DIST_TO_000_CAM1;
		}
	      else if (cam == 2)
		{
		  dist_to_point= DIST_TO_000_CAM2;
		}
	      Pi(0,0) = u*dist_to_point;
	      Pi(1,0) = v*dist_to_point;
	      Pi(2,0) = dist_to_point;
	      Pr(3,0) = 1.;

	    } else if (pp.cnt == 2) // Second point to insert (500mm,0,0) (on the mocap basis)
	    {
	      cv::putText(img,"(1.5,0,0)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = pp.p1[1].x;
	      v = pp.p1[1].y;
	      Eigen::Vector3d undist;
	      undist(0) = u;
	      undist(1) = v;
	      undist(2) = 1.;
	      
	      if (cam == 1)
		{
		  dist_to_point = DIST_TO_0500_CAM1;
		}
	      if (cam == 2)
		{
		  dist_to_point = DIST_TO_0500_CAM2;
		}

	      x = (u)*(dist_to_point);
	      y = (v)*(dist_to_point);

	      Pi(0,1) = x;
	      Pi(1,1) = y;
	      Pi(2,1) = dist_to_point;
	      Pr(0,1) = 1500.;
	      Pr(3,1) = 1.;

	    } else if (pp.cnt == 3) // Third point to insert (0,500mm,0) (on the mocap basis)
	    {
	      cv::putText(img,"(0,1.5,0)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = pp.p1[2].x;
	      v = pp.p1[2].y;
	      Eigen::Vector3d undist;
	      undist(0) = x;
	      undist(1) = y;
	      undist(2) = 1.;
	      
	      if (cam == 1)
		{
		  dist_to_point = DIST_TO_0050_CAM1;
		}
	      if (cam == 2)
		{
		  dist_to_point = DIST_TO_0050_CAM2;
		}
	      x = (u)*(dist_to_point);
	      y = (v)*(dist_to_point);
	      Pi(0,2) = x;
	      Pi(1,2) = y;
	      Pi(2,2) = dist_to_point;
	      Pr(1,2) = 1500.;
	      Pr(3,2) = 1.;

	    } else if (pp.cnt == 4) // Fourth point to insert (0,0,1000mm) (on the mocap basis)
	    {
	      cv::putText(img,"(0,0,1)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = pp.p1[3].x;
	      v = pp.p1[3].y;
	      Eigen::Vector3d undist;
	      undist(0) = u;
	      undist(1) = v;
	      undist(2) = 1.;
	      
	      if (cam == 1)
		{
		  dist_to_point = DIST_TO_001_CAM1;
		}
	      if (cam == 2)
		{
		  dist_to_point = DIST_TO_001_CAM2;
		}

	      x = (u)*(dist_to_point);
	      y = (v)*(dist_to_point);

	      Pi(0,3) = x;
	      Pi(1,3) = y;
	      Pi(2,3) = dist_to_point;
	      Pr(2,3) = 1000.;
	      Pr(3,3) = 1.;
	    }else if (pp.cnt == 5) // Fifth point to insert (1500mm,0,1000mm) (on the mocap basis)
	    {
	      cv::putText(img,"(1.5,0,1)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = pp.p1[4].x;
	      v = pp.p1[4].y;
	      Eigen::Vector3d undist;
	      undist(0) = u;
	      undist(1) = v;
	      undist(2) = 1.;
	      
	      if (cam == 1)
		{
		  dist_to_point = 4856.439;
		}
	      if (cam == 2)
		{
		  dist_to_point = 6333.64;
		}

	      x = (u)*(dist_to_point);
	      y = (v)*(dist_to_point);

	      Pi(0,4) = x;
	      Pi(1,4) = y;
	      Pi(2,4) = dist_to_point;
	      Pr(0,4) = 1500.;
	      Pr(2,4) = 1000.;
	      Pr(3,4) = 1.;
	    }else if (pp.cnt == 6) // sixth point to insert (0,1500mm,1000mm) (on the mocap basis)
	    {
	      cv::putText(img,"(0,1.5,1)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = pp.p1[5].x;
	      v = pp.p1[5].y;
	      Eigen::Vector3d undist;
	      undist(0) = u;
	      undist(1) = v;
	      undist(2) = 1.;
	      
	      if (cam == 1)
		{
		  dist_to_point = 5142.713;
		}
	      if (cam == 2)
		{
		  dist_to_point = 4389.19;
		}

	      x = (u)*(dist_to_point);
	      y = (v)*(dist_to_point);

	      Pi(0,5) = x;
	      Pi(1,5) = y;
	      Pi(2,5) = dist_to_point;
	      Pr(1,5) = 1500.;
	      Pr(2,5) = 1000.;
	      Pr(3,5) = 1.;
	    }
	  cnt = pp.cnt;
	  pp.clicked = false;
	}
      // Keep showing the image and the modification on it
      cv::imshow("Extrinsic Image", img);
      cv::waitKey(10);
    }

  // Compute the rotation matrix and the translation vector
  Eigen::MatrixXd Prinv, Rt;
  pseudoInverse(Pr,Prinv);
  Rt.noalias() = (K.inverse())*Pi*(Prinv);

  rotation(0,0) = Rt(0,0);rotation(0,1) = Rt(0,1);rotation(0,2) = Rt(0,2);
  rotation(1,0) = Rt(1,0);rotation(1,1) = Rt(1,1);rotation(1,2) = Rt(1,2);
  rotation(2,0) = Rt(2,0);rotation(2,1) = Rt(2,1);rotation(2,2) = Rt(2,2);
  translation(0) = Rt(0,3);
  translation(1) = Rt(1,3);
  translation(2) = Rt(2,3);

  // Save the values in files
  if (cam == 1)
    {
      if (!fprintMatrix(rotation, "rotation_cam1.txt"))
	std::cout << "Error writing in rotation_cam1.txt" << std::endl;
      if (!fprintMatrix(translation, "translation_cam1.txt"))
	std::cout << "Error writing in rotation_cam1.txt" << std::endl;
      if(!fprintPT(pan,tilt, "PT_cam1.txt"))
	std::cout << "Error writing file PT_cam1.txt" << std::endl;
    } else if (cam == 2)
    {
      if (!fprintMatrix(rotation, "rotation_cam2.txt"))
	std::cout << "Error writing in rotation_cam2.txt" << std::endl;
      if (!fprintMatrix(translation, "translation_cam2.txt"))
	std::cout << "Error writing in rotation_cam2.txt" << std::endl;
      if(!fprintPT(pan,tilt, "PT_cam2.txt"))
	std::cout << "Error writing file PT_cam2.txt" << std::endl;
    }

  initial_rotation.noalias() = rotation;
  initial_translation.noalias() = translation;
  rotation_computed = true;

  // Destroy the window
  cv::destroyWindow("Extrinsic Image");
}

//\fn void ExtrinsicParam::changePanTilt(double pan, double tilt);
///\brief This function computes the new rotation matrix and the new translation vector of the extrinsic parameters when the camera has changed its position.
///\param pan Value of the new camera panoramique.
///\param tilt Value of the new camera tilt.
void ExtrinsicParam::changePanTilt(double pan, double tilt)
{
  // Compute the rotation matrices with the new values of pan and tilt
  Eigen::Matrix3d Rx, Ry;
  Rx.setZero();
  Ry.setZero();
  Rx(0,0) = 1;
  Rx(1,1) = cos((-(tilt-this->tilt))*PI/180.);
  Rx(1,2) = -sin((-(tilt-this->tilt))*PI/180.);
  Rx(2,1) = sin((-(tilt-this->tilt))*PI/180.);
  Rx(2,2) = cos((-(tilt-this->tilt))*PI/180.);
  Ry(0,0) = cos((-(pan-this->pan))*PI/180.);
  Ry(0,2) = sin((-(pan-this->pan))*PI/180.);
  Ry(1,1) = 1;
  Ry(2,0) = -sin((-(pan-this->pan))*PI/180.);
  Ry(2,2) = cos((-(pan-this->pan))*PI/180.);

  Eigen::Vector3d Tx, Ty;
  Tx.setZero();
  Ty.setZero();
  Tx(0) = 2*3.3*sin(0.5*(this->pan-pan)*PI/180.)*cos(0.5*(this->pan-pan)*PI/180.);
  Tx(2) = -2*3.3*sin(0.5*(this->pan-pan)*PI/180.)*cos((90.-0.5*(this->pan-pan))*PI/180.);
  Ty(1) = 2*3.3*sin(0.5*(this->tilt-tilt)*PI/180.)*cos(0.5*(this->tilt-tilt)*PI/180.);
  Ty(2) = -2*3.3*sin(0.5*(this->tilt-tilt)*PI/180.)*cos((90.-0.5*(this->tilt-tilt))*PI/180.);

  // Compute the new values of the extrinsic parameters
  Eigen::Matrix4d Rx1, Ry1, Rt;
  Rt << initial_rotation, initial_translation,
    0,0,0,1;
  Rx1 << Rx, Tx,
    0,0,0,1;
  Ry1 << Ry, Ty,
    0,0,0,1;
  Rt.noalias() = Rx1*Ry1*Rt;
  rotation(0,0) = Rt(0,0);rotation(0,1) = Rt(0,1);rotation(0,2) = Rt(0,2);
  rotation(1,0) = Rt(1,0);rotation(1,1) = Rt(1,1);rotation(1,2) = Rt(1,2);
  rotation(2,0) = Rt(2,0);rotation(2,1) = Rt(2,1);rotation(2,2) = Rt(2,2);
  translation(0) = Rt(0,3);
  translation(1) = Rt(1,3);
  translation(2) = Rt(2,3);
}

//\fn void ExtrinsicParam::getCameraPointFrom3d(Eigen::Vector3d realP, double &x, double &y, double &z);
///\brief This function computes the coordinates of a 3D point from the real landmark to the one of the camera.
///\param realP Value of the 3D point in the real landmark.
///\param x Value of the 3D point in the camera landmark and on the x axis.
///\param y Value of the 3D point in the camera landmark and on the y axis.
///\param z Value of the distance between the camera and the 3D point (the scale).
void ExtrinsicParam::getCameraPointFrom3d(Eigen::Vector3d realP, double &x, double &y, double &z)
{
  Eigen::Vector4d real;
  real << realP,
          1;
  double fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);  
  Eigen::Vector3d imgP;
  Eigen::MatrixXd Rt;
  Rt.resize(3,4);
  Rt << rotation,translation;
  imgP.noalias() = K*Rt*real;
  z = imgP(2);
  imgP.noalias() = imgP/imgP(2);
  Eigen::Vector3d temp = imgP;
  temp(0) = (temp(0)-cx)/fx;
  temp(1) = (temp(1)-cy)/fy;
  temp.noalias() = distortPoint(temp);
  x = temp(0);
  y = temp(1);
}

//\fn Eigen::Vector3d ExtrinsicParam::undistortPoint(Eigen::Vector3d distortedPoint);
///\brief This function undistorts a distorted point on the image coordinates. After calling this function, the first value of the returned vector should be multiplied by fx and added to cx and the second value should be multiplied by fy and added to cy.
///\param distortedPoint the distorted point to undistort.
///\return The value of the undistorted point in the image coordinates.
Eigen::Vector3d ExtrinsicParam::undistortPoint(Eigen::Vector3d distortedPoint)
{
  Eigen::Vector3d res;
  
  // Convert the camera matrix to an OpenCV matrix
  cv::Mat K1(3,3,CV_64F);
  K1.at<double>(0,0) = K(0,0);
  K1.at<double>(0,1) = K(0,1);
  K1.at<double>(0,2) = K(0,2);
  K1.at<double>(1,0) = K(1,0);
  K1.at<double>(1,1) = K(1,1);
  K1.at<double>(1,2) = K(1,2);
  K1.at<double>(2,0) = K(2,0);
  K1.at<double>(2,1) = K(2,1);
  K1.at<double>(2,2) = K(2,2);

  // Convert the distortion vector to an OpenCV vector
  cv::Mat distCoeffs(5,1,CV_64F);
  distCoeffs.at<double>(0,0) = dist(0);
  distCoeffs.at<double>(1,0) = dist(1);
  distCoeffs.at<double>(2,0) = dist(2);
  distCoeffs.at<double>(3,0) = dist(3);
  distCoeffs.at<double>(4,0) = dist(4);

  // Convert the distorted point vector to an OpenCV vector
  cv::Mat src(1,1,CV_64FC2);
  src.at<double>(0,0) = distortedPoint(0);
  src.at<double>(0,1) = distortedPoint(1);
  cv::Mat dst;

  // Undistort the point
  cv::undistortPoints(src, dst, K1, distCoeffs);

  // Fill the undistorted point vector and return it
  res(0) = dst.at<double>(0,0);
  res(1) = dst.at<double>(0,1);
  res(2) = 1.;
  return res;
}

//\fn Eigen::Vector3d ExtrinsicParam::distortPoint(Eigen::Vector3d undistortedPoint);
///\brief This function distorts an undistorted point on the image coordinates. Before calling this function, the first value of "undistortedPoint" vector should be minused by cx and the whole divided by fx. Besides, the second value of this vector should be minused by cy and the whole divided by fy.
///\param undistortedPoint The undistorted point to distort.
///\return The value of the distorted point in the image coordinates.
Eigen::Vector3d ExtrinsicParam::distortPoint(Eigen::Vector3d undistortedPoint)
{
  Eigen::Vector3d res = undistortedPoint;
  double r2 = res(0)*res(0) + res(1)*res(1);
  double k1 = dist(0), k2 = dist(1), p1 = dist(2), p2 = dist(3), k3 = dist(4);

  // radial distorsion
  res(0) = undistortedPoint(0) * (1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);
  res(1) = undistortedPoint(1) * (1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);

  // tangential distorsion
  res(0) += 2.*p1*undistortedPoint(1)*undistortedPoint(0) + p2*(r2 + 2.*undistortedPoint(0)*undistortedPoint(0));
  res(1) += p1*(r2 + 2.*undistortedPoint(1)*undistortedPoint(1)) + 2.*p2*undistortedPoint(0)*undistortedPoint(1);

  // ideal coordinates => actual coordinates
  res(0) = res(0)*K(0,0) + K(0,2);
  res(1) = res(1)*K(1,1) + K(1,2);
  
  return res;
}

//\fn Eigen::Vector4d triangulate(Point2f p1, Point2f p2, ExtrinsicParam e1, ExtrinsicParam e2);
///\brief This function triangulate a point in 3D.
///\param p1 The first point detected on the first image (cam1).
///\param p2 The second point detected on the second image (cam 2).
///\param e1 The first extrinsic object (cam1).
///\param e2 The second extrinsic object (cam2).
///\return The value of the triangulated point.
Eigen::Vector4d triangulate(cv::Point2f p1, cv::Point2f p2, ExtrinsicParam e1, ExtrinsicParam e2)
{
  std::cout << "p1 = " << p1 << "\tp2 = " << p2 << std::endl;
  Eigen::Matrix3d K = e1.getCameraMatrix(), K1 = e2.getCameraMatrix();
  Eigen::MatrixXd Rt, Rt1;
  Rt.resize(3,4);
  Rt << e1.getRotationMatrix(), e1.getTranslationVector();
  Rt1.resize(3,4);
  Rt1 << e2.getRotationMatrix(), e2.getTranslationVector();
  Eigen::MatrixXd KRt, KRt1;
  KRt.noalias() = K*Rt;
  KRt1.noalias() = K1*Rt1;
  double p1x = (p1.x-K(0,2))/K(0,0);
  double p2x = (p2.x-K1(0,2))/K1(0,0);
  double p1y = (p1.y-K(1,2))/K(1,1);
  double p2y = (p2.y-K1(1,2))/K1(1,1);
  Eigen::VectorXd dist, dist1;
  dist.resize(5);
  dist1.resize(5);
  dist << -0.133553,
    0.078593,
    0.001123,
    0.001457,
    -0.043746;
  dist1 << -0.113323,
	-0.023496,
	-0.013891,
	-0.003838,
	0.267853;
  double r2 = (p1x)*(p1x) + (p1y)*(p1y);
  double xu1 = (p1x)*(1+dist(0)*r2+dist(1)*r2*r2+dist(4)*r2*r2*r2) + dist(2)*2*(p1x)*(p1y) + dist(3)*(r2+2*(p1x)*(p1x));
  double yu1 = (p1y)*(1+dist(0)*r2+dist(1)*r2*r2+dist(4)*r2*r2*r2) + dist(3)*2*(p1x)*(p1y) + dist(2)*(r2+2*(p1y)*(p1y));
  r2 = (p2x)*(p2x) + (p2y)*(p2y);
  double xu2 = (p2x)*(1+dist(0)*r2+dist(1)*r2*r2+dist(4)*r2*r2*r2) + dist(2)*2*(p2x)*(p2y) + dist(3)*(r2+2*(p2x)*(p2x));
  double yu2 = (p2y)*(1+dist(0)*r2+dist(1)*r2*r2+dist(4)*r2*r2*r2) + dist(3)*2*(p2x)*(p2y) + dist(2)*(r2+2*(p2y)*(p2y));
  //std::cout << "xu1=" << xu1 << "\tyu1=" << yu1 << std::endl;
  Eigen::Vector3d P1, P2;
  P1(0) = p1.x;
  P1(1) = p1.y;
  P1(2) = 1.;
  P2(0) = p2.x;
  P2(1) = p2.y;
  P2(2) = 1.;
  P1.noalias() = e1.undistortPoint(P1);
  P2.noalias() = e2.undistortPoint(P2);
  
  Eigen::Matrix4d A;
  /*A << (xu1)*Rt.row(2) - Rt.row(0),
    (yu1)*Rt.row(2) - Rt.row(1),
    (xu2)*Rt1.row(2) - Rt1.row(0),
    (yu2)*Rt1.row(2) - Rt1.row(1);*/
  A << (P1(0))*Rt.row(2) - Rt.row(0),
    (P1(1))*Rt.row(2) - Rt.row(1),
    (P2(0))*Rt1.row(2) - Rt1.row(0),
    (P2(1))*Rt1.row(2) - Rt1.row(1);
  Eigen::Matrix4d At, AtA;
  At.noalias() = A.transpose();
  AtA.noalias() = At*A;
  Eigen::MatrixXd AInv;
  pseudoInverse(A,AInv);
  Eigen::EigenSolver<Eigen::Matrix4d> es(AtA);
  double init = false;
  Eigen::Vector4d min;
  double indexOfMin;
  for (int i=0; i < es.eigenvectors().cols(); i++)
    {
      if (es.eigenvectors().col(i).imag().isApprox(Eigen::Vector4d::Zero()))
	{
	  Eigen::Vector4d real = es.eigenvectors().col(i).real();
	  double one = real(3);
	  real = (1./one)*real;
	  if (!init)
	    {
	      min.noalias() = real;
	      indexOfMin = i;
	      init = true;
	    }
	  else if (es.eigenvalues()[i].real() < es.eigenvalues()[indexOfMin].real())
	    {
	      min.noalias() = real;
	      indexOfMin = i;
	    }
	}
    }
  std::cout << "triangulated point =\n" << min << std::endl;
  return min;
}
