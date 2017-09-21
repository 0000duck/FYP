#ifndef _DATA_TYPE_
#define _DATA_TYPE_

struct Point2D
{
      float u, v;

      Point2D( float u_, float v_ )
      {
	      	u = u_;
	      	v = v_;
      }
      
      void set( float u_, float v_ )
      {
	      	u = u_;
	      	v = v_;
      }

      Point2D(){}

      ~Point2D(){}
};

struct Point3D
{
      float x, y, z;

      Point3D(float x_, float y_, float z_)
      {
	      	x = x_;
	      	y = y_;
	      	z = z_;
      }
      
      void set(float x_, float y_, float z_)
      {
	      	x = x_;
	      	y = y_;
	      	z = z_;
      }

      Point3D(){}

      ~Point3D(){}
};

#endif