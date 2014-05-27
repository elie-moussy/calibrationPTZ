#include "calibrationPTZ/Calibration.hh"
#include <math.h>
#include <limits.h>

std::vector<KeyPoint> kp1[SETS_SIZE][SETS_SIZE], kp2[SETS_SIZE][SETS_SIZE]; // keypoints for matching features between all zoom set images

///\fn void MatToEigen3d(Mat m, Eigen::Matrix3d &e);
///\brief This function copies a 3*3 matrix of opencv (cv::Mat) to an eigen 3*3 double matrix.
///\param m Input matrix of opencv.
///\param e Output matrix of eigen.
void MatToEigen3d(Mat m, Eigen::Matrix3d &e)
{
  for(int i=0; i<3; i++)
    {
      for(int j=0; j<3; j++)
	{
	  e(i,j) = m.at<double>(i,j);
	}
    }
}

///\fn void MatToEigen3d(Mat m, Eigen::Matrix3d &e)Calibration::Calibration(int c);
///\brief This is the constructor of the class "Calibration".
///\param c Input parameter that specifies the camera number (1 or 2).
Calibration::Calibration(int c, SStream* stream) :
  cam(c)
{
  zoomSet = new Mat[SETS_SIZE]; 
  ptSet = new Mat[SETS_SIZE];
  str = stream;
}

///\fn void Calibration::~Calibration();
///\brief This is the destructor of the class "Calibration".
Calibration::~Calibration()
{
  delete [] zoomSet;
  delete [] ptSet;
}

///\fn void Calibration::computePTSet(Mat img, int i);
///\brief This function of the class "Calibration" get an image for the pan-tilt set.
///\param img Input matrix containing the image that need to be stored.
///\param i Input variable that specifies the table index where the image should be stored.
void Calibration::computePTSet(Mat img, int i)
{
  if (i == 0)
    cout << "Starting the acquisation of pan-tilt set !\n" << endl;
  
  ptSet[i] = img.clone();
  ostringstream o;
  o << "pt" << i << ".jpg";
  imwrite(o.str(), img);
  cout << i+1 <<" images acquired out of " << SETS_SIZE << endl;
  if (i == SETS_SIZE-1)
    {
      cout << "\nPan-tilt set acquisation finished !\n" << endl;
    }
}

///\fn void Calibration::computeZoomSet(Mat img, int i);
///\brief This function of the class "Calibration" get an image for the zoom set.
///\param img Input matrix containing the image that need to be stored.
///\param i Input variable that specifies the table index where the image should be stored.
void Calibration::computeZoomSet(Mat img, int i)
{
  if ( i == 0)
    cout << "Starting the acquisation of zoom set !\n" << endl;
  
  zoomSet[i] = img.clone();
  ostringstream o;
  o << "zoom" << i << ".jpg";
  imwrite(o.str(), img);
  cout << i + 1 <<" images acquired out of " << SETS_SIZE << endl;
  if (i == SETS_SIZE-1)
    {
      cout << "\nZoom set acquisation finished !\n" << endl;
    }
}

void Calibration::computeCalibration()
{
  SStream stream;
  if (cam == 2)
    stream.stream = new StreamRTSP("rtsp://root:axis0@axis-ptz2/axis-media/media.amp");
  if (cam == 1)
    stream.stream = new StreamRTSP("rtsp://root:axis0@axis-ptz1/axis-media/media.amp");

  ControlPTZ ctrlPTZ;

  if (cam == 2)
    {
      ctrlPTZ.HTTPRequestPTZPosAbsolute(-31.8904, -20, 0, 2);
      cv::waitKey(1000);
    }
  else
    {
      ctrlPTZ.HTTPRequestPTZPosAbsolute(140, -25, 0, 1);
      cv::waitKey(1000);
    }

  pthread_t thread_img;
  pthread_create(&thread_img, NULL, pthread_img, &stream);

  cv::namedWindow("Calibration",0);
  cv::waitKey(10); // wait 10 ms

  cv::Mat imgBuf;
  bool cont = true;
  int i=0;
  double cx, cy, alpha;
  double fx[SETS_SIZE];
  double fy[SETS_SIZE];
  double k[SETS_SIZE];
  double af, bf, ak, bk;
  Eigen::Matrix3d K[SETS_SIZE];

  while (cont)
    {
      pthread_mutex_lock(&stream.mutex_stock);
      imgBuf = stream.imgBuf.clone();
      pthread_mutex_unlock(&stream.mutex_stock);

      cv::imshow("Calibration",imgBuf);
      cv::waitKey(10); // wait 10 ms

      if (i < SETS_SIZE)
	{
	  computePTSet(imgBuf,i);
	  if (i < 2)	    
	    ctrlPTZ.HTTPRequestPTZPosRelative(3.,0.,0, cam);
	  else
	    ctrlPTZ.HTTPRequestPTZPosRelative(2.,2.,0, cam);
	  cv::waitKey(1000); // wait 1 sec
	  i++;
	} else if (i < 2*SETS_SIZE)
	{
	  computeZoomSet(imgBuf,i-SETS_SIZE);
	  ctrlPTZ.HTTPRequestPTZ(0,0,50,cam);
	  cv::waitKey(1000); // wait 1 sec
	  i++;
	} else if (i == 2*SETS_SIZE)
	{
	  cout << "Starting the computation of the principal point !\n" << endl;
	  computePrincipalPoint(cx,cy);
	  if (cx>704 || cx<1 || cy>576 || cy<1 || isnan(cx) || isnan(cy) || cx == numeric_limits<double>::infinity() || cy == numeric_limits<double>::infinity())
	    {
	      cout << "Failed to compute the principal point." << endl;
	      cout << endl << "Restarting the calibration..." << endl << endl;
	      if (cam == 2)
		{
		  ctrlPTZ.HTTPRequestPTZPosAbsolute(-31.8904, -20, 0, 2);
		  cv::waitKey(1000);
		}
	      else
		{
		  ctrlPTZ.HTTPRequestPTZPosAbsolute(140, -25, 0, 1);
		  cv::waitKey(1000);
		}
	      i=0;
	    }
	  else
	    {
	      cout << "Principal point computation succeded !\n" << endl;
	      i++;
	    }
	} else if (i == 2*SETS_SIZE+1)
	{
	  cout << "Starting the computation of the lens distortion !\n" << endl;
	  computeLensDistortion(k);
	  if (isnan(k[0]) || k[0] == numeric_limits<double>::infinity())
	    {
	      cout << "Failed to compute the lens distortion." << endl;
	      cout << endl << "Restarting the calibration..." << endl << endl;
	      ctrlPTZ.HTTPRequestPTZPosAbsolute(-31.8904, -20, 0);
	      cv::waitKey(1000);
	      i=0;
	    }
	  else
	    {
	      cout << "Lens distortion computation succeded !\n" << endl;
	      i++;
	    }
	} else if (i == 2*SETS_SIZE+2)
	{
	  computeFocalLength(K, fx, fy);
	  if (isnan(fx[0]) || isnan(fy[0]) || fx[0] == numeric_limits<double>::infinity() || fy[0] == numeric_limits<double>::infinity())
	    {
	      cout << "Failed to compute the focal length." << endl;
	      cout << endl << "Restarting the calibration..." << endl << endl;
	      if (cam == 2)
		{
		  ctrlPTZ.HTTPRequestPTZPosAbsolute(-31.8904, -20, 0, 2);
		  cv::waitKey(1000);
		}
	      else
		{
		  ctrlPTZ.HTTPRequestPTZPosAbsolute(140, -25, 0, 1);
		  cv::waitKey(1000);
		}
	      i=0;
	    }
	  else
	    {
	      cout << "Focal length computation succeded !\n" << endl;
	      i++;
	    }
	} else if (i == 2*SETS_SIZE+3)
	{
	  computeZoomScaleDependence(af, bf, ak, bk, alpha, 50);
	  if (isnan(af) || isnan(bf) || isnan(ak) || isnan(bk) || isnan(alpha) || af == numeric_limits<double>::infinity() || bf == numeric_limits<double>::infinity() || ak == numeric_limits<double>::infinity()
	      || bk == numeric_limits<double>::infinity() || alpha == numeric_limits<double>::infinity())
	    {
	      cout << "Failed to compute the zoom scale dependency." << endl;
	      cout << endl << "Restarting the calibration..." << endl << endl;
	      if (cam == 2)
		{
		  ctrlPTZ.HTTPRequestPTZPosAbsolute(-31.8904, -20, 0, 2);
		  cv::waitKey(1000);
		}
	      else
		{
		  ctrlPTZ.HTTPRequestPTZPosAbsolute(140, -25, 0, 1);
		  cv::waitKey(1000);
		}
	      i=0;
	    }
	  else
	    {
	      cout << "Zomm scale dependency computation succeded !\n" << endl;
	      i++;
	    }
	} else if (i == 2*SETS_SIZE+4)
	{
	  cout << "Computing a non linear optimization !\n" << endl;
	  nonLinearOptimization();
	  i++;
	  cout << "Non linear optimization succeded !\n" << endl;
	  cont = false;
	}
    }
  pthread_cancel(thread_img);
  pthread_join(thread_img, NULL);
  delete stream.stream;
}

///\fn void Calibration::computePrincipalPoint(double &cx, double &cy);
///\brief This function of the class "Calibration" computes the principal point of the camera using the zoom set images.
///\param cx Output value of the principal point on the x axis.
///\param cy Output value of the principal point on the y axis.
void Calibration::computePrincipalPoint(double &cx, double &cy)
{
  // c*A = b with c the principal point (vector 2*1), A a matrix 2*n and b a vector 1*n
  Eigen::MatrixXd A;
  Eigen::MatrixXd b;

  int col = 0; // variable helpfull for filling A and b
  int size = 0; // number of points computed wile matching features

  // Match features for the zoom set
  for (int i=0; i < SETS_SIZE; i++)
    {
      for (int j=i+1; j < SETS_SIZE; j++)
	{
	  MatchFeaturesZoom(kp1[i][j], kp2[i][j], i, j);
	  if (j==i+1)
	    size += kp1[i][j].size();
	}
    }

  // Resize A and b with the number of points computed from matching features
  A.resize(2, size);
  b.resize(1,size);

  // Fill A and b thanks to the points computed from matching features
  for (int i=0; i < SETS_SIZE-1 && col < size; i++)
    {
      for (unsigned int k = 0; k < kp1[i][i+1].size() && col < size; k++)
	{
	  A(0,col) = kp1[i][i+1].at(k).pt.y - kp2[i][i+1].at(k).pt.y;
	  A(1,col) = kp2[i][i+1].at(k).pt.x - kp1[i][i+1].at(k).pt.x;
	  b(0,col) = (kp2[i][i+1].at(k).pt.x * kp1[i][i+1].at(k).pt.y) - (kp1[i][i+1].at(k).pt.x * kp2[i][i+1].at(k).pt.y);
	  col++;
	}
    }

  // Compute the pseudo inverse AInv of A
  Eigen::MatrixXd AInv, temp;
  const Eigen::MatrixXd Ac = A;
  pseudoInverse(A, AInv);
  
  // Compute the focal point c = b*Ainv
  temp.noalias() = b*AInv;
  c(0) = temp(0,0);
  c(1) = temp(0,1);
  cx = c(0);
  cy = c(1);
}

///\fn void Calibration::computeLensDistortion(double *k);
///\brief This function of the class "Calibration" computes the lens distortion of the camera using the Pan-Tilt set images.
///\param k Output pointer to the value of the lens distortion.
void Calibration::computeLensDistortion(double *k)
{
  Eigen::Vector2d xd[3], xpd[3]; // coordinates in two images of a computed with matching features
  Eigen::MatrixXd M0,M1,M2; // Matrices that will help to determine the lens distortion
  M0.resize(5,5);
  M1.resize(5,5);
  M2.resize(5,5);
  M0.fill(0.);
  M1.fill(0.);
  M2.fill(0.);

  double s, sp; // norm of vectors xd and xpd
  double minor; // the minor value for the minimization problem
  Point2f C; // the focal point
  C.x = this->c(0);
  C.y = this->c(1);

  bool init = false; // variable helpfull for the minimization problem
  
  this->kz = k;
  kz[0] = 0.; // lens distortion
  
  std::vector<KeyPoint> kp1, kp2; // keypoints of the matching features
  MatchFeaturesPT(kp1,kp2,0,1);
  
  unsigned int m = 0; // variable helpfull to fill the matrices M0, M1 and M2 and vectors xd and xpd
  while ( m < kp1.size()-3)
    {
      // Fill M0, M1, M2, xd and xpd
      const int t = m+3;
      for (int i = m; i < t; i++)
	{
	  xd[i-m](0) = kp1.at(m).pt.x - C.x;
	  xd[i-m](1) = kp1.at(m).pt.y - C.y;
	  xpd[i-m](0) = kp2.at(m).pt.x - C.x;
	  xpd[i-m](1) = kp2.at(m).pt.y - C.y;
	  s = sqrt(xd[i-m](0)*xd[i-m](0) + xd[i-m](1)*xd[i-m](1));
	  sp = sqrt(xpd[i-m](0)*xpd[i-m](0) + xpd[i-m](1)*xpd[i-m](1));
	  
	  M0((i-m),2) = 0.-xpd[i-m](1);
	  M0((i-m),3) = xd[i-m](1)*xpd[i-m](0);
	  M0((i-m),4) = xd[i-m](1);
	  M0((i-m)+1,0) = xpd[i-m](0);
	  M0((i-m)+1,1) = 1.;
	  M0((i-m)+1,3) = 0.-xd[i-m](0)*xpd[i-m](0);
	  M0((i-m)+1,4) = 0.-xd[i-m](0);
	  
	  M1((i-m),2) = s*xpd[i-m](1);
	  M1((i-m),4) = sp*xd[i-m](1);
	  M1((i-m)+1,0) = s*xpd[i-m](0);
	  M1((i-m)+1,1) = sp+s;
	  M1((i-m)+1,4) = 0.-sp*xd[i-m](0);
	  
	  M2((i-m)+1,1) = sp*s;
	}
      xd[2](0) = kp1.at(t-1).pt.x - C.x;
      xd[2](1) = kp1.at(t-1).pt.y - C.y;
      xpd[2](0) = kp2.at(t-1).pt.x - C.x;
      xpd[2](1) = kp2.at(t-1).pt.y - C.y;
      s = sqrt(xd[2](0)*xd[2](0) + xd[2](1)*xd[2](1));
      sp = sqrt(xpd[2](0)*xpd[2](0) + xpd[2](1)*xpd[2](1));
      
      M0(4,2) = 0.-xpd[2](1);
      M0(4,3) = xd[2](1)*xpd[2](0);
      M0(4,4) = xd[2](1);
      
      M1(4,2) = s*xpd[2](1);
      M1(4,4) = sp*xd[2](1);
      m = t-1;
      
      // Compute the principal point for the above matrices
      Eigen::MatrixXd h; // vector that will contain the non zero values of the homography matrix
      Eigen::VectorXd lambda; // the values computed for the lens distortion
      polyeig(M0, M1, M2, h, lambda);
      
      // Compute a non linear minimization in order to get the best value of the lens distortion
      Eigen::Matrix3d H; // Homography matrix between two points computed with the matching feature
      H.fill(0.);
      double res; // variable helpfull for the minimization
      
      for (int i=0; i < h.cols(); i++)
	{
	  // Fill the Homography matrix
	  H(0,0) = h(0,i);
	  H(0,2) = h(1,i);
	  H(1,1) = h(2,i);
	  H(2,0) = h(3,i);
	  H(2,2) = h(4,i);
	  
	  // Minimization
	  Eigen::MatrixXd Mat;
	  for (unsigned int j=0; j<kp1.size()-3; j=j+3)
	    {
	      // Fill the points
	      Eigen::MatrixXd x, xp;
	      x.resize(2,3);
	      xp.resize(3,2);
	      x(0,0) = kp1.at(j).pt.x;
	      x(0,1) = kp1.at(j+1).pt.x;
	      x(0,2) = kp1.at(j+2).pt.x;
	      x(1,0) = kp1.at(j).pt.y;
	      x(1,1) = kp1.at(j+1).pt.y;
	      x(1,2) = kp1.at(j+2).pt.y;
	      xp(0,0) = kp2.at(j).pt.x;
	      xp(1,0) = kp2.at(j+1).pt.x;
	      xp(2,0) = kp2.at(j+2).pt.x;
	      xp(0,1) = kp2.at(j).pt.y;
	      xp(1,1) = kp2.at(j+1).pt.y;
	      xp(2,1) = kp2.at(j+2).pt.y;

	      double norm, normp; // norm of x and xp
	      
	      // Minimization
	      for(int k=1; k<=10; k++)
		{
		  if (k!=1)
		    {
		      x = x.cwiseProduct(x);
		      xp = xp.cwiseProduct(xp);
		    }
		  norm = x.norm();
		  normp = xp.norm();
		  Mat.noalias() = (x/(1+lambda(i)*norm))*H*(xp/(1+lambda(i)*normp));
		  res += Mat.norm();
		}
	    }
	  if (!init) // case minor is not initialized
	    {
	      kz[0] = lambda(i);
	      minor = res;
	      init =true;
	    }
	  else if (res < minor /*&& fabs(lambda(i)) < 1 && fabs(lambda(i)) > 1e-6*/) // case we find a better lens distortion
	    {
	      kz[0] = lambda(i);
	      minor = res;
	    }
	}  
    }
}

///\fn void Calibration::computeFocalLength(Eigen::Matrix3d* K0, double* fx, double* fy);
///\brief This function of the class "Calibration" computes the focal length of the camera using the Pan-Tilt set images.
///\param K0 Output pointer to the calibration matrix.
///\param fx Output pointer to the value of the focal length on the x axis.
///\param fy Output pointer to the value of the focal length on the y axis.
void Calibration::computeFocalLength(Eigen::Matrix3d* K0, double* fx, double* fy)
{
  Eigen::Matrix3d Heig; // Homography matrix with eigen
  Mat Hcv; // Homography matrix with opencv

  Point2f C; // The principal point
  C.x = c(0);
  C.y = c(1);

  double cnt = 0; // number of good fxi and fyi computed

  K = K0;
  this->fx = fx;
  this->fy = fy;

  double w3_w1 = 0., w3_w2 = 0.; // w(3)/w(1) and w(3)/w(2) with w(1), w(2) and w(3) the diagonal values of omega(0)
  
  for (int i=1; i < SETS_SIZE; i++)
    {
      for (int j=i+1; j < SETS_SIZE; j++)
	{
	  // Point detection with the SURF algorithm
	  std::vector<KeyPoint> kp1, kp2;
	  MatchFeaturesPT(kp1,kp2,i,j);
	  
	  // Create undistorted points
	  std::vector< Point2f > v1, v2;
	  for (unsigned int k = 0; k < kp1.size() && k < kp2.size(); k++)
	    {
	      Point2f pt1, pt2;
	      pt1.x = (kp1.at(k).pt.x - C.x)/(1+kz[0]*((kp1.at(k).pt.x - C.x)*(kp1.at(k).pt.x - C.x) + (kp1.at(k).pt.y - C.y)*(kp1.at(k).pt.y - C.y)));
	      pt1.y = (kp1.at(k).pt.y - C.y)/(1+kz[0]*((kp1.at(k).pt.x - C.x)*(kp1.at(k).pt.x - C.x) + (kp1.at(k).pt.y - C.y)*(kp1.at(k).pt.y - C.y)));
	      pt2.x = (kp2.at(k).pt.x - C.x)/(1+kz[0]*((kp2.at(k).pt.x - C.x)*(kp2.at(k).pt.x - C.x) + (kp2.at(k).pt.y - C.y)*(kp2.at(k).pt.y - C.y)));
	      pt2.y = (kp2.at(k).pt.y - C.y)/(1+kz[0]*((kp2.at(k).pt.x - C.x)*(kp2.at(k).pt.x - C.x) + (kp2.at(k).pt.y - C.y)*(kp2.at(k).pt.y - C.y)));
	      v1.push_back(pt1);
	      v2.push_back(pt2);
	    }
	  
	  // Find Homography Matrix between two undistorted images
	  Hcv = findHomography(v1,v2,CV_RANSAC);
	  MatToEigen3d(Hcv, Heig);
	  
	  // Find fxi and fyi using the homography with the equation : (H.transpose())*w(0) = w(0)*(H.inverse())
	  Eigen::Matrix3d Heigi, Heigt;
	  Heigi = Heig.inverse();
	  Heigt = Heig.transpose();
	  if (fabs(Heigt(2,0)/Heigi(0,2)) > 1e-1 && fabs(Heigt(2,1)/Heigi(1,2)) > 1e-1 && fabs(Heigt(2,0)/Heigi(0,2)) < 2. && fabs(Heigt(2,1)/Heigi(1,2)) < 2.)
	    {
	      w3_w1 += fabs(Heigt(2,0)/Heigi(0,2));
	      w3_w2 += fabs(Heigt(2,1)/Heigi(1,2));
	      cnt++;
	    }
	}
    }
  
  // Find fx0 and fy0 using the mean value of fxi and fyi
  fx[0] = fabs(w3_w1/cnt);
  fy[0] = fabs(w3_w2/cnt);
  
  // Compute K0 with fx0 and fy0
  K[0].setZero();
  K[0](0,0) = sqrt(cnt/w3_w1);
  K[0](1,1) = sqrt(cnt/w3_w2);
  K[0](2,2) = 1;
}

///\fn void Calibration::computeZoomScaleDependence(double &af, double &bf, double &ak, double &bk, double &alpha, int stepZoom);
///\brief This function of the class "Calibration" computes the focal as function of the zoom, some parameters (af, bf, ak, bk) helpfull for the calibration and ratio between the focal projection on the x axis and the one the y axis.
///\param af Output reference to the parameter af.
///\param bf Output reference to the parameter bf.
///\param ak Output reference to the parameter ak.
///\param bk Output reference to the parameter bk.
///\param alpha Output reference to the value of the ratio between the focal projection on the x axis and the one the y axis.
///\param stepZoom Input value helpfull to get the zoom step.
void Calibration::computeZoomScaleDependence(double &af, double &bf, double &ak, double &bk, double &alpha, int stepZoom)
{
  // Compute the lens distortion kz(z) for each zoom
  Point2f C; // Principal point
  C.x = c(0);
  C.y = c(1);

  // Set all the lens distortions to 0.
  for (int i=1; i < SETS_SIZE; i++)
    kz[i] = 0.;

  // Compute the lens distortions
  for (int i=1; i < SETS_SIZE; i++)
    {
      unsigned int k = 0;
      Eigen::MatrixXd h; // Vector that contains the non zero values of the homography matrix
      Eigen::Matrix3d H; // Homography matrix between the undistorted image at the lowest zoom scale and the distorted one at a higher zoom scale
      Eigen::VectorXd lambda; // Vector that will contain the lens distortion for a zoom scale

      double minor; // variable helpfull for the minimization
      bool init = false; // variable helpfull for the minimization

      
      while (kz[i] == 0. && k < kp1[0][i].size()-4)
	{
	  init = false;
	  const int t = k + 3;
 
	  std::vector< Point2f > v1, v2;
	  Point2f pt1[kp1[0][i].size()], pt2[kp2[0][i].size()];
	  Eigen::Matrix3d M1, M2; // Matrices helpfull to compute the lens distortion and the homography matrix
	  M1.setZero();
	  M2.setZero();
	  double si; // norm of point2

	  // Fill the points
	  for (int j = k; j < t; j++)
	    {
	      pt1[k].x = (kp1[0][i].at(k).pt.x - C.x)/(1+kz[0]*((kp1[0][i].at(k).pt.x - C.x)*(kp1[0][i].at(k).pt.x - C.x) + (kp1[0][i].at(k).pt.y - C.y)*(kp1[0][i].at(k).pt.y - C.y)));
	      pt1[k].y = (kp1[0][i].at(k).pt.y - C.y)/(1+kz[0]*((kp1[0][i].at(k).pt.x - C.x)*(kp1[0][i].at(k).pt.x - C.x) + (kp1[0][i].at(k).pt.y - C.y)*(kp1[0][i].at(k).pt.y - C.y)));
	      pt2[k] = kp2[0][i].at(k).pt;
	      k++;
	    }

	  // Fill the matrices
	  si = sqrt( (pt2[t-3].x)*(pt2[t-3].x) + (pt2[t-3].y)*(pt2[t-3].y) );
	  M1(0,1) = 0.-pt2[t-3].y;
	  M1(0,2) = pt1[t-3].y;
	  M1(1,0) = pt2[t-3].x;
	  M1(1,2) = 0.-pt1[t-3].x;
	  M1(2,1) = 0.-pt2[t-2].y;
	  M1(2,2) = pt1[t-2].y;
	  
	  M2(0,1) = 0. - si*pt1[t-3].y;
	  M2(0,1) = si*pt1[t-3].x;
	  si = sqrt( (pt2[t-2].x)*(pt2[t-2].x) + (pt2[t-2].y)*(pt2[t-2].y) );
	  M2(2,1) = 0. - si*pt1[t-2].y;

	  // Compute h and lambda
	  polyeig(M1, M2, h, lambda);
	  
	  // Compute a non linear minimization in order to get the best value of the lens distortion
	  Eigen::Matrix3d H; // Homography matrix between two points computed with the matching feature
	  H.setZero();
	  double res; // variable helpfull for the minimization
	  
	  // Fill the Homography matrix
	  H(0,0) = h(0,0)/h(2,0);
	  H(1,1) = h(1,0)/h(2,0);
	  H(2,2) = h(2,0)/h(2,0);
	      
	  // Minimization
	  Eigen::MatrixXd Mat;
	  for (unsigned int j=0; j<kp1[0][i].size()-3; j=j+3)
	    {
	      // Fill the points
	      Eigen::MatrixXd x, xp;
	      x.resize(2,3);
	      xp.resize(3,2);
	      x(0,0) = kp1[0][i].at(j).pt.x;
	      x(0,1) = kp1[0][i].at(j+1).pt.x;
	      x(0,2) = kp1[0][i].at(j+2).pt.x;
	      x(1,0) = kp1[0][i].at(j).pt.y;
	      x(1,1) = kp1[0][i].at(j+1).pt.y;
	      x(1,2) = kp1[0][i].at(j+2).pt.y;
	      xp(0,0) = kp2[0][i].at(j).pt.x;
	      xp(1,0) = kp2[0][i].at(j+1).pt.x;
	      xp(2,0) = kp2[0][i].at(j+2).pt.x;
	      xp(0,1) = kp2[0][i].at(j).pt.y;
	      xp(1,1) = kp2[0][i].at(j+1).pt.y;
	      xp(2,1) = kp2[0][i].at(j+2).pt.y;
		  
	      double norm, normp; // norm of x and xp
		  
	      // Minimization function
	      for(int l=1; l<=10; l++)
		{
		  if (l!=1)
		    {
		      x = x.cwiseProduct(x);
		      xp = xp.cwiseProduct(xp);
		    }
		  norm = x.norm();
		  normp = xp.norm();
		  Mat.noalias() = (x/(1+lambda(0)*norm))*H*(xp/(1+lambda(0)*normp));
		  res += Mat.norm();
		}
	      if (!init && fabs(lambda(0)) > kz[i-1]) // case minor is not initialized
		{
		  kz[i] = lambda(0);
		  minor = res;
		  (K[i]).setZero();
		  (K[i])(0,0) = H(0,0)*(K[0])(0,0);
		  (K[i])(1,1) = H(1,1)*(K[0])(1,1);
		  (K[i])(2,2) = H(2,2)*(K[0])(2,2);
		  K[i] = K[i](2,2)*K[i];
		  fx[i] = fabs((K[i])(0,0)/(K[i])(2,2));
		  fy[i] = fabs((K[i])(1,1)/(K[i])(2,2));
		  init =true;
		}
	      else if (res < minor && fabs(lambda(0)) > kz[i-1]) // case we find a better lens distortion
		{
		  kz[i] = lambda(0);
		  minor = res;
		}
	    }
	}
    }

  // Compute focal length as function of zoom
  const double dx = fx[0]/3.3, dy = fy[0]/3.3;  // [3.3,119] is the focal length range according to the camera datasheet
  double fi;
  double z = 0.;

  for(int i=0;i<SETS_SIZE-1;i++)
    {
      z += stepZoom;
      fi = 0.011571157*z + 3.3;
      fx[i+1] = dx*fi;
      fy[i+1] = dy*fi;
      cout << "fx[" << i+1 << "] = " << fx[i+1] << endl;
      cout << "fy[" << i+1 << "] = " << fy[i+1] << endl;
    }
  
  // Find aspect ratio (alpha)
  alpha = 0.;
  double cpt = 0.;
  for (int i=0; i < SETS_SIZE; i++)
    {
      alpha += fy[i]/fx[i];
      cpt += 1.;
    }
  alpha = alpha / cpt;
  this->alpha = alpha;
  
  cout << "alpha = " << alpha << endl;
  
  // Find af and bf
  Eigen::MatrixXd abf;
  Eigen::MatrixXd zoom;
  zoom.resize(2,SETS_SIZE-1);
  Eigen::MatrixXd focal;
  focal.resize(1,SETS_SIZE-1);  
  for (int i=0; i < SETS_SIZE-1; i++)
    {
      zoom(0,i) = (i+1)*stepZoom;
      zoom(1,i) = zoom(0,i) * zoom(0,i);
      focal(0,i) = fx[i+1] - fx[0];
    }
  Eigen::MatrixXd zoomInv;
  pseudoInverse(zoom, zoomInv,1e-6);
  abf.noalias() = focal*zoomInv;
  af = abf(0,0);
  this->af = af;
  bf = abf(0,1);
  this->bf = bf;
  
  cout << "af = " << af << endl;
  cout << "bf = " << bf << endl;
  
  // Find ak and bk
  Eigen::MatrixXd abk;
  Eigen::MatrixXd temp;
  temp.resize(2,SETS_SIZE-1);
  Eigen::MatrixXd fk;
  fk.resize(1,SETS_SIZE-1);
  for (int i=0; i < SETS_SIZE-1; i++)
    {
      temp(0,i) = 1.;
      temp(1,i) = 0. - sqrt(fabs(kz[i+1]-kz[0]));
      focal(0,i) = fx[i+1]*sqrt(fabs(kz[i+1]-kz[0]));
    }
  Eigen::MatrixXd tempInv;
  pseudoInverse(temp,tempInv,1e-6);
  abk.noalias() = fk*tempInv;
  ak = abk(0,0)*abk(0,0);
  this->ak = ak;
  bk = abk(0,1);
  this->bk = bk;
  
  cout << "ak = " << ak << endl;
  cout << "bk = " << bk << endl;
}

///\fn void optFunc(double *p, double *x, int m, int n, void *data);
///\brief This function computes the value of the mathematical function of the final non-linear minimization of the calibration.
///\param p Input/Output pointer to the parameters that need to be adjusted by the minimization.
///\param x Output pointer to the value of the function.
///\param m Input variable containing the size of the parameter p.
///\param n Input variable containing the size of the parameter x.
void optFunc(double *p, double *x, int m, int n, void *data)
{
	// p is the table containing the variables (in this case 8)
	// optFunc(af, bf, fx[0], ak, bk, kz[0], cx, cy)
	// x is the function result
	Eigen::Matrix3d H;
	double results = 0.;

	for (int i=0; i<SETS_SIZE; i++)
	{
		int zi = (i+1)*100;	// 100 = stepZoom
		for (int j=i+1; j<SETS_SIZE; j++)
		{
			int zj = (i+1)*100;
		
			// Compute the homography matrix
			H.setZero();
			H(0,0) = H(1,1) = 1. + (p[0]*zi+p[1]*zi*zi)/(p[2]+p[0]*zj+p[1]*zj*zj);
			H(2,2) = 0.;
			H(0,2) = p[6] * (1. - H(1,1));
			H(1,2) = p[7] * (1. - H(1,1));
		
			// Create undistorted points from zoom features
			Eigen::Vector3d pt1, pt2;
			for (unsigned int k=0; k<kp1[i][j].size();k++)
			{
				Eigen::Vector2d temp1, temp2;
				temp1(0) = kp1[i][j].at(k).pt.x;
				temp1(1) = kp1[i][j].at(k).pt.y;
				temp2(0) = kp2[i][j].at(k).pt.x;
				temp2(1) = kp2[i][j].at(k).pt.y;
				pt1(2) = pt2(2) = 1.;
				pt1(0) = temp1(0)/( 1. + (temp1.norm()*temp1.norm()+1.)*(p[5] + p[3]/((p[2]+p[0]*zi+p[1]*zi*zi+p[4])*(p[2]+p[0]*zi+p[1]*zi*zi+p[4]))) );
				pt1(1) = temp1(1)/( 1. + (temp1.norm()*temp1.norm()+1.)*(p[5] + p[3]/((p[2]+p[0]*zi+p[1]*zi*zi+p[4])*(p[2]+p[0]*zi+p[1]*zi*zi+p[4]))) );
				pt2(0) = temp2(0)/( 1. + (temp1.norm()*temp1.norm()+1.)*(p[5] + p[3]/((p[2]+p[0]*zi+p[1]*zi*zi+p[4])*(p[2]+p[0]*zi+p[1]*zi*zi+p[4]))) );
				pt2(1) = temp2(1)/( 1. + (temp1.norm()*temp1.norm()+1.)*(p[5] + p[3]/((p[2]+p[0]*zi+p[1]*zi*zi+p[4])*(p[2]+p[0]*zi+p[1]*zi*zi+p[4]))) );
			
				// Computing results
				Eigen::Vector3d vect;
				vect.noalias() = H*pt1 - pt2;
				results += vect.norm() * vect.norm();
			}
		}
	}

	// storing the results in x
	for(int i = 0; i < n; i++)
		x[i] = results;	
}

///\fn void Calibration::nonLinearOptimization();
///\brief This function of the class "Calibration" adjust the values computed during the calibration thanks to a final non-linear minimization.	
void Calibration::nonLinearOptimization()
{
	double P[8], x[8]; 
	int m = 8, n = 8;
	double info[LM_INFO_SZ];

	P[0] = af; P[1] = bf; P[2] = fx[0]; P[3] = ak; P[4] = bk; P[5] = kz[0]; P[6] = c(0); P[7] = c(1);
	
	dlevmar_dif(optFunc, P, x, m, n, 1000, NULL, info, NULL, NULL, NULL);
	cout << "af = " << P[0] << endl << "bf = " << P[1] << endl;
	cout << "fx[0] = " << P[2] << endl << "ak = " << P[3] << endl;
	cout << "bk = " << P[4] << endl << "kz[0] = " << P[5] << endl;
	cout << "c(0) = " << P[6] << endl << "c(1) = " << P[7] << endl;
	af = P[0]; bf = P[1];
	fx[0] = P[2]; ak = P[3];
	bk = P[4]; kz[0] = P[5];
	c(0) = P[6]; c(1) = P[7];
}

void Calibration::computeMechanicalError(double &perror, double &terror)
{
  double pan, tilt, panm, tiltm;
  int z;
  ControlPTZ ctrlPTZ;
  srand(time(NULL));
  perror = 0.;
  terror = 0.;
  for (int i = 0; i < 50; i++)
    {
      if (cam == 2)
	{
	  pan = fRand(-100, 100);//26;
	  tilt = fRand(-100, 100); //3;
	  ctrlPTZ.HTTPRequestPTZPosAbsolute(pan, tilt, 0, 2);
	  cv::waitKey(1000);
	  ctrlPTZ.refreshPosition(panm, tiltm, z, 2);
	}
      else
	{
	  pan = fRand(-100, 100);//132;
	  tilt = fRand(-100, 100);//1.3;
	  ctrlPTZ.HTTPRequestPTZPosAbsolute(pan, tilt, 0, 1);
	  cv::waitKey(1000);
	  ctrlPTZ.refreshPosition(panm, tiltm, z, 1);
	}
      perror += panm/pan;
      terror += tiltm/tilt;
    }
  perror /= 50.;
  terror /= 50.;
  pan_error = perror;
  tilt_error = terror;
}

//\fn void Calibration::MatchFeaturesZoom(std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, int i, int j);
///\brief This function computes the features between two images of the zoom set using a surf detector.
///\param kp1 Output reference vector that contains the points detected on the first image (i).
///\param kp2 Output reference vector that contains the points detected on the second image (j).
///\param i Input variable containing the number of the first image.
///\param j Input variable containing the number of the second image.
void Calibration::MatchFeaturesZoom(std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, int i, int j)
{
	//-- Step 1: Detect keypoints using a Surf detector
	int minHessian = 400;

	SurfFeatureDetector detector( minHessian );

	std::vector<KeyPoint> keypoints_1, keypoints_2;
	Mat grey1, grey2;
	cvtColor(ptSet[i], grey1, CV_RGB2GRAY);
	cvtColor(ptSet[j], grey2, CV_RGB2GRAY);

	detector.detect( grey1, keypoints_1 );
	detector.detect( grey2, keypoints_2 );

	//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;

	Mat descriptors_1, descriptors_2;

	extractor.compute( zoomSet[i], keypoints_1, descriptors_1 );
	extractor.compute( zoomSet[j], keypoints_2, descriptors_2 );

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match( descriptors_1, descriptors_2, matches );

	for( int i = 0; i < (int)matches.size(); i++ )
  	{
	  kp1.push_back(keypoints_1[matches[i].queryIdx]);
	  kp2.push_back(keypoints_2[matches[i].trainIdx]);
	}
}

//\fn void Calibration::MatchFeaturesPT(std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, int i, int j);
///\brief This function computes the features between two images of the pan-tilt set using a surf detector.
///\param kp1 Output reference vector that contains the points detected on the first image (i).
///\param kp2 Output reference vector that contains the points detected on the second image (j).
///\param i Input variable containing the number of the first image.
///\param j Input variable containing the number of the second image.
void Calibration::MatchFeaturesPT(std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, int i, int j)
{
	//-- Step 1: Detect keypoints using a Surf detector
	int minHessian = 400;

	SurfFeatureDetector detector( minHessian );

	std::vector<KeyPoint> keypoints_1, keypoints_2;
	Mat grey1, grey2;
	cvtColor(ptSet[i], grey1, CV_RGB2GRAY);
	cvtColor(ptSet[j], grey2, CV_RGB2GRAY);

	detector.detect( grey1, keypoints_1 );
	detector.detect( grey2, keypoints_2 );

	//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;

	Mat descriptors_1, descriptors_2;

	extractor.compute( ptSet[i], keypoints_1, descriptors_1 );
	extractor.compute( ptSet[j], keypoints_2, descriptors_2 );

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match( descriptors_1, descriptors_2, matches );

	for( int i = 0; i < (int)matches.size(); i++ )
  	{
	  kp1.push_back(keypoints_1[matches[i].queryIdx]);
	  kp2.push_back(keypoints_2[matches[i].trainIdx]);
	}
}

