#include "liang-barsky.h"
#include <cmath>

double checkLimits(double x, double l1, double l2);

// Liang-Barsky function by Daniel White @ http://www.skytopia.com/project/articles/compsci/clipping.html
// This function inputs 8 numbers, and outputs 4 new numbers (plus a boolean value to say whether the clipped line is drawn at all).
//
bool LiangBarsky (double edgeLeft, double edgeRight, double edgeBottom, double edgeTop,   // Define the x/y clipping values for the border.
                  double x0src, double y0src, double x1src, double y1src,                 // Define the start and end points of the line.
                  double &x0clip, double &y0clip, double &x1clip, double &y1clip)         // The output values, so declare these outside.
{

    double t0 = 0.0;    double t1 = 1.0;
    double xdelta = x1src-x0src;
    double ydelta = y1src-y0src;
    double p,q,r;

    for(int edge=0; edge<4; edge++) {   // Traverse through left, right, bottom, top edges.
        if (edge==0) {  p = -xdelta;    q = -(edgeLeft-x0src);  }
        if (edge==1) {  p = xdelta;     q =  (edgeRight-x0src); }
        if (edge==2) {  p = -ydelta;    q = -(edgeBottom-y0src);}
        if (edge==3) {  p = ydelta;     q =  (edgeTop-y0src);   }
        r = q/p;
        if(p==0 && q<0) return false;   // Don't draw line at all. (parallel line outside)

        if(p<0) {
            if(r>t1) return false;         // Don't draw line at all.
            else if(r>t0) t0=r;            // Line is clipped!
        } else if(p>0) {
            if(r<t0) return false;      // Don't draw line at all.
            else if(r<t1) t1=r;         // Line is clipped!
        }
    }

    x0clip = checkLimits(x0src + t0*xdelta, edgeLeft, edgeRight);
    y0clip = checkLimits(y0src + t0*ydelta, edgeBottom, edgeTop);
    x1clip = checkLimits(x0src + t1*xdelta, edgeLeft, edgeRight);
    y1clip = checkLimits(y0src + t1*ydelta, edgeBottom, edgeTop);

    return true;        // (clipped) line is drawn
}

double checkLimits(double x, double l1, double l2) {
    double newX = x;
    double epsilon = 5e-15;
    if (fabs(x - l1) < epsilon)
        newX = l1;
    else if (fabs(x - l2) < epsilon)
        newX = l2;
    return newX;
}

