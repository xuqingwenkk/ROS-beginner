#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

typedef struct {
   float x,y,z;
} XYZ;

typedef struct {
   int r,g,b;
} RGB;

typedef struct {
   int axis;
   float value;
   float cvalue,svalue;
} TRANSFORM;

typedef struct {
  Mat rgb;
  int width;
  int height;
  float theta;
} IMAGE;

typedef struct {
  Mat rgb;
  int width;
  int height;
  float theta;
  int cx,cy;
  int radius;
} FISHIMAGE;

typedef struct {
  vector<TRANSFORM> trans;
  int antialias;
} SETTING;

#define XTILT 0 
#define YROLL 1 
#define ZPAN  2 

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

#define DTOR 0.0174532925f
#define RTOD 57.2957795f

#define ABS(x) (x < 0 ? -(x) : (x))
#define SIGN(x) (x < 0 ? (-1) : 1)
#define MODULUS(p) (sqrt(p.x*p.x + p.y*p.y + p.z*p.z))

// Prototypes
void CameraRay(float x,float y,XYZ *p,IMAGE & img);
XYZ VectorSum(float,XYZ,float,XYZ,float,XYZ,float,XYZ);
XYZ operator-(XYZ p1, XYZ p2);
void pixel_map(IMAGE & persp, FISHIMAGE & fishimage, SETTING & set);

class ParamReader
{
public:
  map<string, string> data;
public:
  ParamReader(string filename="../param/parameters.txt")
  {
    ifstream fin(filename.c_str());
    if(!fin)
    {
      cerr << "param file does not exist! " << endl;
      return;
    }
    while(!fin.eof())
    {
      string str;
      getline(fin,str);
      if(str[0]=='#')
	continue;
      int pos=str.find("=");
      if(pos == -1)
	continue;
      string key = str.substr(0, pos);
      string value = str.substr(pos+1, str.length());
      data[key] = value;
      if(!fin.good())
	break;
    }
  }
  string getData(string key)
  {
    map<string, string>::iterator iter = data.find(key);
    if(iter == data.end())
    {
      cerr << key << "is not found !" << endl;
      return string("NOT FOUND! ");
    }
    return iter->second;
  }
};

