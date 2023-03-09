#include "aruco_3d/aruco_3d.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>// aruco functionalities
#include <opencv2/aruco/dictionary.hpp>

#include <tf/transform_datatypes.h>

#include <std_msgs/String.h>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <iostream>

namespace aruco_3dgrid {

/***********************************************************************************************************************
 * Class definitions: Aruco3dGridEstimator
 */
/***********************************************************
 * Primary methods
 */
    Aruco3dGridEstimator::Aruco3dGridEstimator() {
    }

    // Estimator function
    Aruco3dGridEstimator::Result Aruco3dGridEstimator::PoseEstimate(std::vector<tf::Transform> T)
    {
        double dist[T.size()-1];

        Aruco3dGridEstimator::Result result;

        Aruco3dGridEstimator::PCoord rot;

        Aruco3dGridEstimator::PCoord pAcum; pAcum.x = 0.00; pAcum.y = 0.00; pAcum.z = 0.00;
        Aruco3dGridEstimator::PCoord rotAcum; rotAcum.x = 0.00; rotAcum.y = 0.00; rotAcum.z = 0.00;
        Aruco3dGridEstimator::PCoord pEstimated; pEstimated.x = 0.00; pEstimated.y = 0.00; pEstimated.z = 0.00;
        Aruco3dGridEstimator::PCoord rotEstimated; rotEstimated.x = 0.00; rotEstimated.y = 0.00; rotEstimated.z = 0.00;
     
        if (T.size() > 2)// In case more than two markers are detected (central + various non-central)
        {       
            std::vector<double> pXFiltered, pYFiltered, pZFiltered;
            pXFiltered.resize(T.size()-2); pYFiltered.resize(T.size()-2); pZFiltered.resize(T.size()-2);
            std::vector<double> rotXFiltered, rotYFiltered, rotZFiltered;
            rotXFiltered.resize(T.size()-2); rotYFiltered.resize(T.size()-2); rotZFiltered.resize(T.size()-2);          

            // Obtain absolute distances between estimated positions and central tag
            for (unsigned int i = 1; i < T.size(); i++)
            {
                dist[i-1] = GetRelDistance(T.at(0), T.at(i));
            }
                
            // Get maximum of distances and filter them
            Aruco3dGridEstimator::MaxValue max = FindMax(dist, T.size()-1);
            printf("Max value: %f \n", max.value);

            // Filter coordinates
            int index = 0;

            for (unsigned int i = 1; i < T.size(); i++)
            {
                printf("Evaluating marker %i. Distance %f \n", i, dist[i-1]);
            
                if (dist[i-1] != max.value)
                {
                    // Filtered positions
                    pXFiltered.at(index) = T.at(i).getOrigin().x();
                    pYFiltered.at(index) = T.at(i).getOrigin().y();
                    pZFiltered.at(index) = T.at(i).getOrigin().z();
                    
                    // Accumulate positions
                    pAcum.x += T.at(i).getOrigin().x();
                    pAcum.y += T.at(i).getOrigin().y();
                    pAcum.z += T.at(i).getOrigin().z();

                    // Filtered orientations
                    rot = GetAngles(T.at(i));
                    rotXFiltered.at(index) = rot.x;
                    rotYFiltered.at(index) = rot.y;
                    rotZFiltered.at(index) = rot.z;
                    
                    index++;
                }
            }
            //printf("Filtering finished. Marker idx %i filtered\n", max.index+1);

            // Correct positive-negative rotations
            rotXFiltered = AlphaCorrectSigns(rotXFiltered, T.size()-2);
            rotYFiltered = CorrectSigns(rotYFiltered, T.size()-2);
            rotZFiltered = CorrectSigns(rotZFiltered, T.size()-2);
            
            // Accumulate rotations
            for (unsigned int i = 0; i < T.size()-2; i++)
            {
                rotAcum.x += rotXFiltered.at(i);
                rotAcum.y += rotYFiltered.at(i);
                rotAcum.z += rotZFiltered.at(i);
            }
            
            // Compute estimations
            pEstimated.x = pAcum.x / (T.size()-2);
            pEstimated.y = pAcum.y / (T.size()-2);
            pEstimated.z = pAcum.z / (T.size()-2);            

            rotEstimated.x = rotAcum.x / (T.size()-2);
            rotEstimated.y = rotAcum.y / (T.size()-2);
            rotEstimated.z = rotAcum.z / (T.size()-2);
            
            // Compute measurement content value
            result.content = EvalContent(pXFiltered, pYFiltered, pZFiltered, T.size()-2);
            printf("Content various markers: %f\n", result.content);

        }
        else
        {
            for (int i = 0; i < T.size(); i++)
            {
                pEstimated.x += GetCoord(T.at(i), "x");
                pEstimated.y += GetCoord(T.at(i), "y");
                pEstimated.z += GetCoord(T.at(i), "z");
            
                pEstimated.x = pEstimated.x / T.size();
                pEstimated.y = pEstimated.y / T.size();
                pEstimated.z = pEstimated.z / T.size();
                //result.pEstimated = pEstimated;
                
                rot = GetAngles(T.at(i));
                rotAcum.x += rot.x;
                rotAcum.y += rot.y;
                rotAcum.z += rot.z;
                
                rotEstimated.x = rotAcum.x / T.size();
                rotEstimated.y = rotAcum.y / T.size();
                rotEstimated.z = rotAcum.z / T.size();
                //result.rotEstimated = rotEstimated;
            }
                
            result.content = 0.00;
            printf("Content: %f\n", result.content);

            
        }

        tf::Matrix3x3 resRot;
        resRot.setRPY(rotEstimated.x, rotEstimated.y, rotEstimated.z);
        tf::Vector3 resPos(pEstimated.x, pEstimated.y, pEstimated.z);

        tf::Transform Tresult(resRot, resPos);

        result.T = Tresult;

        return result;
    }

    tf::Transform Aruco3dGridEstimator::Matrix2Tf(double m[][4])
    {
        tf2::Matrix3x3 tf_rot(m[1][1], m[1][2], m[1][3],
                              m[2][1], m[2][2], m[2][3],
                              m[3][1], m[3][2], m[3][3]);
        tf2::Vector3 tf_orig(m[1][4], m[2][4], m[3][4]);

        tf2::Transform tf2_tf(tf_rot, tf_orig);

        return tf::Transform(tf::Matrix3x3(tf::Quaternion(tf2_tf.getRotation().x(),
                                           tf2_tf.getRotation().y(),
                                           tf2_tf.getRotation().z(),
                                           tf2_tf.getRotation().w())),
                             tf::Vector3(tf2_tf.getOrigin().x(),
                                         tf2_tf.getOrigin().y(),
                                         tf2_tf.getOrigin().z()));
    }

    double Aruco3dGridEstimator::GetRelDistance(tf::Transform Tini, tf::Transform Tend)
    {
       
        double x = abs(Tend.getOrigin().x() - Tini.getOrigin().x());
        double y = abs(Tend.getOrigin().y() - Tini.getOrigin().y());
        double z = abs(Tend.getOrigin().z() - Tini.getOrigin().z());

        return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    }

    Aruco3dGridEstimator::MaxValue Aruco3dGridEstimator::FindMax(double d[], int n)
    {
        Aruco3dGridEstimator::MaxValue max;
        
        max.value = *std::max_element(d, d + n);
        
        int index = 0;

        for (int i = 0; i < n; i++)
        {
            if (d[i] == max.value)
            {
                max.index = i;
            }
        }

    return max;
    }

    float Aruco3dGridEstimator::EvalContent(std::vector<double> valX, std::vector<double> valY, std::vector<double> valZ, int n)
    {
        Aruco3dGridEstimator::PCoord max; max.x = -100.00; max.y = -100.00; max.z = -100.00;
        Aruco3dGridEstimator::PCoord min; min.x = 100.00; min.y = 100.00; min.z = 100.00;

        for (unsigned int i = 0; i < n; i++)
        {
            // Max values
            if (valX.at(i) >= max.x)
            {
                max.x = valX.at(i);
            }
            if (valY.at(i) >= max.y)
            {
                max.y = valY.at(i);
            }
            if (valZ.at(i) >= max.z)
            {
                max.z = valZ.at(i);
            }
               
            // Min values
            if (valX.at(i) < min.x)
            {
                min.x = valX.at(i);
            }
            if (valY.at(i) < min.y)
            {
                min.y = valY.at(i);
            }
            if (valZ.at(i) < min.z)
            {
                min.z = valZ.at(i);
            }
        }
        return (max.x - min.x) * (max.y - min.y) * (max.z - min.z);
    }

    std::vector<double> Aruco3dGridEstimator::CorrectSigns(std::vector<double> Coord, int n)
    {
        int pos = 0, neg = 0;
        
        for (int i = 0; i < n; i++)
        {
            if (Coord[i] > 0)
            {
                pos++;
            }
            else
            {
                neg++;
            }
        }
        if (pos == neg){
            return Coord;
        }
        else
        {
            if (pos >= neg)
            {
                return SetOrientation(Coord, n, 1);
            }
            else
            {
                return SetOrientation(Coord, n, -1);
            }
        }
    }

    std::vector<double> Aruco3dGridEstimator::AlphaCorrectSigns(std::vector<double> Coord, int n)
    {
        for (int i = 0; i < n; i++)
        {
            if (Coord.at(i) < 0)
            {
                Coord.at(i) = Coord.at(i) + 2 * M_PI;
            }
        }
        return CorrectSigns(Coord, n);
    }

    std::vector<double> Aruco3dGridEstimator::GammaCorrectSigns(std::vector<double> Coord, int n)
    {
        for (int i = 0; i < n; i++)
        {
            if (Coord.at(i) > M_PI)
            {
                Coord.at(i) = -Coord.at(i) + M_PI;
            }
        }
        return CorrectSigns(Coord, n);
    }

    std::vector<double> Aruco3dGridEstimator::SetOrientation(std::vector<double> CoordIn, int n, int ori)
    {
        std::vector<double> CoordOut;
        CoordOut.resize(n);
        
        for (int i = 0; i < n; i++)
        {
            CoordOut.at(i) = ori * abs(CoordIn.at(i));
        }
        return CoordOut;
    }

    double Aruco3dGridEstimator::GetCoord(tf::Transform T, std::string Coord)
    {
        if (Coord == "x")
        {
            return T.getOrigin().x();
        }
        else if (Coord == "y")
        {
            return T.getOrigin().y();
        }
        else if (Coord == "z")
        {
            return T.getOrigin().z();
        }
        else
        {
            return 0.00;
        }
    }

    Aruco3dGridEstimator::PCoord Aruco3dGridEstimator::GetAngles(tf::Transform T)
    {
        Aruco3dGridEstimator::PCoord rot;
        
        tf::Matrix3x3 R = T.getBasis();
        R.getRPY(rot.x, rot.y, rot.z);

        return rot;
    }

};// end aruco_3d namespace