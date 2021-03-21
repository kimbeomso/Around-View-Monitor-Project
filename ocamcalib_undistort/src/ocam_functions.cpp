/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ocamcalib_undistort/ocam_functions.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>

#define M_DEG2RAD  3.1415926 / 180.0

//------------------------------------------------------------------------------
bool get_ocam_model(struct ocam_model *myocam_model, const char *filename)
{
 double *pol        = myocam_model->pol;
 double *invpol     = myocam_model->invpol;
 double *xc         = &(myocam_model->xc);
 double *yc         = &(myocam_model->yc);
 double *c          = &(myocam_model->c);
 double *d          = &(myocam_model->d);
 double *e          = &(myocam_model->e);
 int    *width      = &(myocam_model->width);
 int    *height     = &(myocam_model->height);
 int *length_pol    = &(myocam_model->length_pol);
 int *length_invpol = &(myocam_model->length_invpol);
 FILE *f;
 char buf[CMV_MAX_BUF];
 int i;

 //Open file
 if(!(f=fopen(filename,"r")))
 {
   printf("File %s cannot be opened\n", filename);
   return false;
 }

 //Read polynomial coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_pol);
 for (i = 0; i < *length_pol; i++)
 {
     fscanf(f," %lf",&pol[i]);
 }

 //Read inverse polynomial coefficients
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_invpol);
 for (i = 0; i < *length_invpol; i++)
 {
     fscanf(f," %lf",&invpol[i]);
 }

 //Read center coordinates
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf\n", xc, yc);

 //Read affine coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf %lf\n", c,d,e);

 //Read image size
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d %d", height, width);

 fclose(f);
 return true;
}

//------------------------------------------------------------------------------
void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model)
{
//  double start =ros::Time::now().toSec();
//  double end = ros::Time::now().toSec();
//  std::cout<< "Time : " << end - start << std::endl;

 double *pol    = myocam_model->pol;
 double xc      = (myocam_model->xc);
 double yc      = (myocam_model->yc);
 double c       = (myocam_model->c);
 double d       = (myocam_model->d);
 double e       = (myocam_model->e);
 int length_pol = (myocam_model->length_pol);
 double invdet  = 1/(c-d*e);    // 1/det(A), where A = [c,d;e,1] as in the Matlab file


 double xp = invdet*(    (point2D[0] - xc) - d*(point2D[1] - yc) );
 double yp = invdet*( -e*(point2D[0] - xc) + c*(point2D[1] - yc) );

    // double start =ros::Time::now().toSec();
 double temp = xp*xp + yp*yp;
 double r   = sqrt( temp ); //distance [pixels] of  the point from the image center    // 0.8 - 0.1sec 
 double zp  = pol[0];
 double r_i = 1;
 int i;

 for (i = 1; i < length_pol; i++)                             //0.8 - 0.15 sec 
 {
   r_i *= r;
   zp  += r_i*pol[i];
 }

 //normalize to unit norm
 double invnorm = 1/sqrt( temp + zp*zp );           //0.8 - 0.14 sec

// transform cam Coordinate 
 point3D[0] = invnorm*xp;
 point3D[1] = invnorm*yp;
 point3D[2] = invnorm*zp;
  // double end = ros::Time::now().toSec();
  //  std::cout<< "Time : " << end - start << std::endl;
//  std::cout << point3D << std::endl;
}

//------------------------------------------------------------------------------
void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
 double *invpol     = myocam_model->invpol;
 double xc          = (myocam_model->xc);
 double yc          = (myocam_model->yc);
 double c           = (myocam_model->c);
 double d           = (myocam_model->d);
 double e           = (myocam_model->e);
 int    width       = (myocam_model->width);
 int    height      = (myocam_model->height);
 int length_invpol  = (myocam_model->length_invpol);
 double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
 double theta       = atan(point3D[2]/norm);
 double t, t_i;
 double rho, x, y;
 double invnorm;
 int i;

  if (norm != 0)
  {
    invnorm = 1/norm;
    t  = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i*invpol[i];
    }

    x = point3D[0]*invnorm*rho;
    y = point3D[1]*invnorm*rho;

    point2D[0] = x*c + y*d + xc;
    point2D[1] = x*e + y   + yc;
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
  }

  // std::cout << point2D[0] << point2D[1] <<std::endl;
}
//------------------------------------------------------------------------------
void create_perspecive_undistortion_LUT( CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf)
{
     int i, j;
     int width = mapx->cols; //New width
     int height = mapx->rows;//New height
     float *data_mapx = mapx->data.fl;
     float *data_mapy = mapy->data.fl;
     float Nxc = height/2.0;
     float Nyc = width/2.0;
     float Nz  = -width/sf;
     double M[3];
     double m[2];

     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {
             M[0] = (i - Nxc);
             M[1] = (j - Nyc);
             M[2] = Nz;
             world2cam(m, M, ocam_model);

             *( data_mapx + i*width+j ) = (float) m[1];
             *( data_mapy + i*width+j ) = (float) m[0];
            //  std::cout<<"data_mapx" << data_mapx <<std::endl;
         }
}

//------------------------------------------------------------------------------
void create_panoramic_undistortion_LUT ( CvMat *mapx, CvMat *mapy, float Rmin, float Rmax, float xc, float yc )
{
     int i, j;
     float theta;
     int width = mapx->width;
     int height = mapx->height;
     float *data_mapx = mapx->data.fl;
     float *data_mapy = mapy->data.fl;
     float rho;

     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {
             theta = -((float)j)/width*2*M_PI; // Note, if you would like to flip the image, just inverte the sign of theta
             rho   = Rmax - (Rmax-Rmin)/height*i;
             *( data_mapx + i*width+j ) = yc + rho*sin(theta); //in OpenCV "x" is the
             *( data_mapy + i*width+j ) = xc + rho*cos(theta);
         }
}


// double M_front_param[6] = {0.688 * M_DEG2RAD,  21.631 * M_DEG2RAD,   3.103* M_DEG2RAD   ,1.905,   0.033, 0.707 };
// double M_left_param[6] =  {1.133 * M_DEG2RAD,  19.535 * M_DEG2RAD,   92.160* M_DEG2RAD  ,0.0,     1.034, 0.974 };
// double M_right_param[6] = {3.440 * M_DEG2RAD,  18.273 * M_DEG2RAD,  -86.127* M_DEG2RAD  ,0.0,    -1.034, 0.988 };
// double M_back_param[6] =  {0.752 * M_DEG2RAD,  31.238 * M_DEG2RAD,  -178.189* M_DEG2RAD ,-2.973, -0.065, 0.883 };

XY_coord InvProjGRD(int Ximg,int Yimg,double ROLL,double PITCH, double YAW, double World_CamX,double World_CamY,double World_CamZ, struct ocam_model *myocam_model){
  double unit_vector[3] = {0.0 ,0.0 ,0.0};

  // *** caution *** : in ocam 'x' is rows, 'y' is cols
  double point2D[2]={Yimg,Ximg};

  cam2world(unit_vector, point2D, myocam_model); // -> -0.06sec

  //3D unit vector
  // double unit_vector_new[3] = {-unit_vector[2] , -unit_vector[1], -unit_vector[0]};  // Xc, Yc, Zc
  // double Rotation_Matrix[3][3] = 
  // { { cos(YAW)*cos(PITCH), cos(YAW)*sin(PITCH)*sin(ROLL)-sin(YAW)*cos(ROLL), cos(YAW)*sin(PITCH)*cos(ROLL)+sin(YAW)*sin(ROLL)},
  //   { sin(YAW)*cos(PITCH), sin(YAW)*sin(PITCH)*sin(ROLL)+cos(YAW)*cos(ROLL), sin(YAW)*sin(PITCH)*cos(ROLL)-cos(YAW)*sin(ROLL)},
  //   { -sin(PITCH), cos(PITCH)*sin(ROLL)  , cos(PITCH)*cos(ROLL)} };

  // double Rotation_Matrix[3][3] =                           //left
  // { { -0.035519 , -0.9993493119, 0.0071551457},
  //   { 0.94174032, -0.03107600176, 0.33484421811},
  //   { -0.3344, 0.018631248  , 0.94221152} };

  //double Rotation_Matrix[3][3] =                            //right
  // { { 0.06413905989, 0.997189291, -0.0387259469},
  //   {-0.9474047206  ,0.0486526741,  -0.3163182142},
  //   {-0.3135450149 , 0.0569774977, 0.9478623784} }

  // double World_unit_vec[3] = 
  // {Rotation_Matrix[0][0] * unit_vector_new[0] + Rotation_Matrix[0][1] * unit_vector_new[1] + Rotation_Matrix[0][2] * unit_vector_new[2],
  //  Rotation_Matrix[1][0] * unit_vector_new[0] + Rotation_Matrix[1][1] * unit_vector_new[1] + Rotation_Matrix[1][2] * unit_vector_new[2],
  //  Rotation_Matrix[2][0] * unit_vector_new[0] + Rotation_Matrix[2][1] * unit_vector_new[1] + Rotation_Matrix[2][2] * unit_vector_new[2]};
  double World_unit_vec[3];


  if(YAW == 92.160* M_DEG2RAD){
    World_unit_vec[0] = -0.03552 * -unit_vector[2] -0.9993   * -unit_vector[1] + 0.007155 * -unit_vector[0];
    World_unit_vec[1] = 0.94174 *  -unit_vector[2] -0.031076 * -unit_vector[1] + 0.33484 *  -unit_vector[0];
    World_unit_vec[2] = -0.3344 *  -unit_vector[2] + 0.01863 * -unit_vector[1] + 0.9422 *   -unit_vector[0];
  }
  else if(YAW == -86.127* M_DEG2RAD){
    //3.440 ,  18.273 ,  -86.127
    World_unit_vec[0] = (0.06413905989)*  -unit_vector[2] +  0.997189291* -unit_vector[1]  +  (-0.0387259469)* -unit_vector[0];
    World_unit_vec[1] = (-0.9474047206)*  -unit_vector[2]  + 0.0486526741* -unit_vector[1]  + (-0.3163182142)*  -unit_vector[0];
    World_unit_vec[2] = (-0.3135450149)*  -unit_vector[2]  + ( 0.0569774977)* -unit_vector[1] +  (0.9478623784)*   -unit_vector[0];
  }
  else if(YAW == 3.103* M_DEG2RAD){
    //0.688,  21.631,   3.103
    World_unit_vec[0] = (0.928214264)*  -unit_vector[2]  + (-0.0497073597)* -unit_vector[1]  + (0.3687105349)*  -unit_vector[0];
    World_unit_vec[1] = (0.0503190315)*  -unit_vector[2]  + (0.9987014513)* -unit_vector[1]  + (0.0079628035)*  -unit_vector[0];
    World_unit_vec[2] = (-0.3686275563)*  -unit_vector[2]  + (0.0111619692)* -unit_vector[1]  + (0.9295101587)*  -unit_vector[0];
  }
  else
  {
    //0.752 ,  31.238 ,  -178.189
    World_unit_vec[0] = (-0.8545934306)*  -unit_vector[2]  + (0.0247970386)* -unit_vector[1]  + (-0.5187052874)*  -unit_vector[0];
    World_unit_vec[1] = (-0.0270209137)*  -unit_vector[2]  + (-0.9996295214)* -unit_vector[1]  + (-0.0032695956)*  -unit_vector[0];
    World_unit_vec[2] = (-0.5185941945)*  -unit_vector[2]  + (0.0018265772)* -unit_vector[1]  + (0.8549468607)*  -unit_vector[0];
  }
  

  double Xw;
  double Yw;
  // if  World_unit_vec[2] >= 0  do nothing -> if it is not GRD 
  if(World_unit_vec[2] < 0)
  {
    Xw = ( -World_CamZ ) * World_unit_vec[0] / World_unit_vec[2] + World_CamX;
    Yw = ( -World_CamZ ) * World_unit_vec[1] / World_unit_vec[2] + World_CamY;    
  }
  else{
    Xw = 0;
    Yw = 0;
  }


  XY_coord xy;
  xy.x = Xw;
  xy.y = Yw;

  return xy;

}
