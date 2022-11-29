#ifndef UTIL_H
#define UTIL_H 1
#include <math.h>

#define PI 3.14159265
#define deg2rad(x) x*PI/180.0f
namespace ARCS2{
    struct Vector3{
        Vector3(double x, double y, double z):x(x),y(y),z(z){};
        
        double x=0;
        double y=0;
        double z=0;

        Vector3 operator+(const Vector3 &v){
            return Vector3(x+v.x,y+v.y,z+v.z);
        }

        Vector3 operator-(const Vector3 &v){
            return Vector3(x-v.x,y-v.y,z-v.z);
        }

        void operator=(const Vector3 &v){
            x = v.x;
            y = v.y;
            z = v.z;
        }

        void print(){
            printf("[%.2f, %.2f, %.2f] ",x,y,z);
        }

    };

    struct Matrix3
    {
        Matrix3(){
            ;
        }

        Matrix3(double a,double b,double c,double d,double e,double f, double g, double h, double i){
            arr[0]=a;
            arr[1]=b;
            arr[2]=c;

            arr[3]=d;
            arr[4]=e;
            arr[5]=f;
            
            arr[6]=g;
            arr[7]=h;
            arr[8]=i;
        };
        
        double arr[9]={1,0,0,0,1,0,0,0,1};

        Vector3 operator*(const Vector3 &v){
            return Vector3(arr[0]*v.x+arr[1]*v.y+arr[2]*v.z,
                            arr[3]*v.x+arr[4]*v.y+arr[5]*v.z,
                            arr[6]*v.x+arr[7]*v.y+arr[8]*v.z);
        }

        Matrix3 operator*(const Matrix3 &m){
            return Matrix3(arr[0]*m.arr[0]+m.arr[1]*m.arr[3]+arr[2]*m.arr[6], arr[0]*m.arr[1]+arr[1]*m.arr[4]+arr[2]*m.arr[7], arr[0]*m.arr[2]+arr[1]*m.arr[5]+arr[2]*m.arr[8],
                           arr[3]*m.arr[0]+m.arr[4]*m.arr[3]+arr[5]*m.arr[6], arr[3]*m.arr[1]+arr[4]*m.arr[4]+arr[5]*m.arr[7], arr[3]*m.arr[2]+arr[4]*m.arr[5]+arr[5]*m.arr[8],
                           arr[6]*m.arr[0]+m.arr[7]*m.arr[3]+arr[8]*m.arr[6], arr[6]*m.arr[1]+arr[7]*m.arr[4]+arr[8]*m.arr[7], arr[6]*m.arr[2]+arr[7]*m.arr[5]+arr[8]*m.arr[8]);
        }

        double operator[](int i){
            return arr[i];
        }

        void operator=(const Matrix3 &m){
            arr[0] = m.arr[0];
            arr[1] = m.arr[1];
            arr[2] = m.arr[2];
            arr[3] = m.arr[3];
            arr[4] = m.arr[4];
            arr[5] = m.arr[5];
            arr[6] = m.arr[6];
            arr[7] = m.arr[7];
            arr[8] = m.arr[8];
        }

        inline Matrix3 transpose(){
            return Matrix3(arr[0],arr[3],arr[6],
                            arr[1],arr[4],arr[7],
                            arr[2],arr[5],arr[8]);
        }

        void print(){
            printf("%.2f, %.2f, %.2f \n %.2f, %.2f, %.2f \n %.2f, %.2f, %.2f \n",arr[0],arr[1],arr[2], arr[3],arr[4],arr[5], arr[6],arr[7],arr[8]);
        }
    };

    const Matrix3 I3 = Matrix3(1,0,0,0,1,0,0,0,1);

    template<class T>
    inline Matrix3 T_ZYX(T y,T p,T r){       
        double sy = sin(y);
        double cy = cos(y);
        double sp = sin(p);
        double cp = cos(p);
        double sr = sin(r);
        double cr = cos(r);
        return Matrix3(cy*cp, cy*sp*sr-sy*cr, cy*sp*cr + sy*sr,
                        sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
                        -sp,   cp*sr,          cp*cr);
    }

    template<class T>
    inline Matrix3 T_ZYX_d(T y,T p,T r){
            y = deg2rad(y);
            p = deg2rad(p);
            r = deg2rad(r);
            return T_ZYX(y,p,r);
        }

    struct TransMatrices
    {
        TransMatrices(ARCS2::Matrix3* t_s2e, ARCS2::Matrix3* t_b2s):T_S2E(t_s2e),T_B2S(t_b2s){}
        ARCS2::Matrix3* T_S2E;
        ARCS2::Matrix3* T_B2S;
    };

    struct OpenChain
    {
        Vector3 base = Vector3(0,0,0);
        std::vector<std::pair<Vector3,TransMatrices>> links;
    };



}
#endif