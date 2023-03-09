#ifndef ARUCO_3D_H_
#define ARUCO_3D_H_

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>// aruco functionalities
#include <opencv2/aruco/dictionary.hpp>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>

#include <std_msgs/String.h>
#include <string>

namespace aruco_3dgrid {

class Aruco3dGridEstimator {

    public:
          
        struct MaxValue{
            double value;
            int index;
        };

        struct PCoord{
            double x;
            double y;
            double z;
        };

        struct Result{
            tf::Transform T;
            float content;
        };

        Aruco3dGridEstimator();

        // Function to compute pose
        Result PoseEstimate(std::vector<tf::Transform> T);

        tf::Transform Matrix2Tf(double m[][4]);

    private:

        double GetRelDistance(tf::Transform Tini, tf::Transform Tend);

        MaxValue FindMax(double d[], int i);

        float EvalContent(std::vector<double> valX, std::vector<double> valY, std::vector<double> valZ, int n);

        std::vector<double> CorrectSigns(std::vector<double> Coord, int n);

        std::vector<double> AlphaCorrectSigns(std::vector<double> Coord, int n);

        std::vector<double> GammaCorrectSigns(std::vector<double> Coord, int n);

        std::vector<double> SetOrientation(std::vector<double> CoordIn, int n, int ori);

        double GetCoord(tf::Transform T, std::string Coord);

        PCoord GetAngles(tf::Transform T);
};

}// namespace aruco_3d
#endif // ARUCO_3D_H