#include <iostream>
#include <ros/ros.h>
#include <iterator>
#include <vector>
#include <string> 

#include <aruco/aruco.h>
#include <aruco_3d/aruco_3d.h>
#include <aruco/cvdrawingutils.h>
#include <aruco_ros/aruco_ros_utils.h>

#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/TransformArray.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

class Aruco3D
{
    private:

        // ROS pubs/subs:
        ros::NodeHandle nh;
        
        image_transport::ImageTransport it;
        tf::TransformListener _tfListener;

        image_transport::Subscriber image_sub;
        
        ros::Subscriber cam_info_sub;
        bool cam_info_received;

        image_transport::Publisher image_pub;
        image_transport::Publisher debug_pub;

        ros::Publisher pose_pub;
        ros::Publisher transform_pub;
        ros::Publisher transforms_pub;
        ros::Publisher position_pub;
        ros::Publisher transform_grid_pub;
        ros::Publisher pose_grid_pub;
        ros::Publisher position_grid_pub;
        ros::Publisher marker_pub; // rviz visualization marker
        ros::Publisher marker_list_pub;
        ros::Publisher pixel_pub;

        // Aruco stuff
        cv::Mat inImage;
        cv::Mat outImage;
        aruco::CameraParameters camParam;
        bool useRectifiedImages, normalizeImageIllumination;
        int dctComponentsToRemove;

        aruco::MarkerDetector mDetector;// Standard ArUco detector
        aruco_3dgrid::Aruco3dGridEstimator gridEstimator;// 3D grid ArUco estimator

        tf::StampedTransform rightToLeft;
        std::string marker0_frame;
        std::string marker1_frame;
        std::string marker2_frame;
        std::string marker3_frame;
        std::string marker4_frame;
        std::string camera_frame;
        std::string reference_frame;
               
        dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;     

        aruco_msgs::MarkerArray::Ptr markers_msg;
        aruco_msgs::TransformArray::Ptr transforms_msg;

        std::vector<tf::Transform> T;

        std::vector<aruco::Marker> markers;
        std::vector<geometry_msgs::TransformStamped> transforms;

        double marker_size;
        int marker0_id, marker1_id, marker2_id, marker3_id, marker4_id;

        aruco_3dgrid::Aruco3dGridEstimator::Result poseEstimation;

        // Transformation matrices of Aruco tags (from central tag to each tag)
        double m0[4][4] = {{1.00, 0.00, 0.00, 0.00},
                           {0.00, 1.00, 0.00, 0.00},
                           {0.00, 0.00, 1.00, 0.00},
                           {0.00, 0.00, 0.00, 1.00}};

        double m1[4][4] = {{0.969, 0.00, -0.259, -0.08},
                           {0.00, 1.00, 0.00, 0.00},
                           {0.256, 0.00, 0.966, 0.013},
                           {0.00, 0.00, 0.00, 1.00}};
        
        double m2[4][4] = {{0.00, 1.00, 0.00, -0.08},
                           {-0.966, 0.00, 0.256, 0.00},
                           {0.259, 0.00, 0.966, -0.013},
                           {0.00, 0.00, 0.00, 1.00}};
    
        double m3[4][4] = {{-0.966, 0.00, 0.259, -0.08},
                           {0.00, -1.00, 0.00, 0.00},
                           {0.259, 0.00, 0.966, -0.013},
                           {0.00, 0.00, 0.00, 1.00}};

        double m4[4][4] = {{0.00, -1.00, 0.00, -0.08},
                           {0.966, 0.00, 0.259, 0.00},
                           {0.259, 0.00, 0.966, -0.013},
                           {0.00, 0.00, 0.00, 1.00}};
    public:

        Aruco3D() :
            cam_info_received(false), nh("~"), it(nh)
        {
            if (nh.hasParam("corner_refinement"))
                ROS_WARN("Corner refinement options have been removed in ArUco 3.0.0, corner_refinement ROS parameter is deprecated");

            // Detection settings
            aruco::MarkerDetector::Params params = mDetector.getParameters();
            
            // STILL NOT IMPLEMENTED
            std::string thresh_method;
            switch (params._thresMethod)
            {
            case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
                thresh_method = "THRESH_ADAPTIVE";
                break;
            case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
                thresh_method = "THRESH_AUTO_FIXED";
                break;
            default:
                thresh_method = "UNKNOWN";
                break;
            }
            
            ROS_INFO_STREAM("Threshold method: " << thresh_method);

            float min_marker_size; // percentage of image area
            nh.param<float>("min_marker_size", min_marker_size, 0.02);
                   
            std::string detection_mode;
            nh.param<std::string>("detection_mode", detection_mode, "DM_FAST");
            if (detection_mode == "DM_FAST")
                mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
            else if (detection_mode == "DM_VIDEO_FAST")
                mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
            else
                mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);
                       
            //ROS_INFO_STREAM("Detection mode: " << detection_mode);
            ROS_INFO_STREAM("Marker size min: " << min_marker_size << "% of image area");

            // ROS pubs-subs
            image_sub = it.subscribe("/image", 1, &Aruco3D::image_callback, this);// General topic
            cam_info_sub = nh.subscribe("/camera_info", 1, &Aruco3D::cam_info_callback, this);// General topic
            
            image_pub = it.advertise("result", 1);
            debug_pub = it.advertise("debug", 1);

            pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
            transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
            transforms_pub = nh.advertise<aruco_msgs::TransformArray>("transforms", 100);
            position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
            transform_grid_pub = nh.advertise<geometry_msgs::TransformStamped>("transform_grid",100);
            pose_grid_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_grid", 100);
            position_grid_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position_grid", 100);
            marker_pub = nh.advertise<aruco_msgs::MarkerArray>("markers", 100);
            marker_list_pub = nh.advertise<std_msgs::UInt32MultiArray>("markers_list", 10);
            pixel_pub = nh.advertise<geometry_msgs::PointStamped>("pixel", 10);

            normalizeImageIllumination = false;

            // Params
            nh.param<double>("marker_size", marker_size, 0.06);
            nh.param<std::string>("reference_frame", reference_frame, "");
            nh.param<std::string>("camera_frame", camera_frame, "");
            nh.param<std::string>("marker0_frame", marker0_frame, "");
            nh.param<std::string>("marker1_frame", marker1_frame, "");
            nh.param<std::string>("marker2_frame", marker2_frame, "");
            nh.param<std::string>("marker3_frame", marker3_frame, "");
            nh.param<std::string>("marker4_frame", marker4_frame, "");
            //nh.getParam("markers_id", markers_id);
            nh.param<int>("marker0_id", marker0_id, 0);
            nh.param<int>("marker1_id", marker1_id, 1);
            nh.param<int>("marker2_id", marker2_id, 2);
            nh.param<int>("marker3_id", marker3_id, 3);
            nh.param<int>("marker4_id", marker4_id, 4);
            nh.param<bool>("normalizeImage", normalizeImageIllumination, true);
            nh.param<bool>("image_is_rectified", useRectifiedImages, true);
            nh.param<int>("dct_components_to_remove", dctComponentsToRemove, 2);
            if (dctComponentsToRemove == 0)
                normalizeImageIllumination = false;
            nh.param<bool>("image_is_rectified", useRectifiedImages, true);
            ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);

            // Verify definition of frames
            //ROS_ASSERT(camera_frame != "" && marker0_frame != "" && marker1_frame != "" && marker2_frame != "" && marker3_frame != "" && marker4_frame != "");

            if (reference_frame.empty())
            reference_frame = camera_frame;

            markers_msg = aruco_msgs::MarkerArray::Ptr(new aruco_msgs::MarkerArray());
            markers_msg->header.frame_id = reference_frame;
            markers_msg->header.seq = 0;

            transforms_msg = aruco_msgs::TransformArray::Ptr(new aruco_msgs::TransformArray());
            transforms_msg->header.frame_id = reference_frame;
          
            // Print info of ArUco marker
            ROS_INFO("ArUco node started with marker size of %f m and marker0 id: %d", marker_size, marker0_id);
            ROS_INFO("ArUco node will publish central pose to TF with <%s> as parent and <%s> as child.", reference_frame.c_str(), marker0_frame.c_str());

            dyn_rec_server.setCallback(boost::bind(&Aruco3D::reconf_callback, this, _1, _2));
        }

        bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform)
        {
            std::string errMsg;

            if (!_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &errMsg))
            {
                ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
                return false;
            }
            else
            {
                try
                {
                    _tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), transform); // get latest available
                }
                catch (const tf::TransformException& e)
                {
                    ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
                    return false;
                }
            }
            return true;
        }

        void image_callback (const sensor_msgs::ImageConstPtr& msg)
        {
            // Check pubs
            if ((image_pub.getNumSubscribers() == 0) && (debug_pub.getNumSubscribers() == 0)
                && (pose_pub.getNumSubscribers() == 0) && (transform_pub.getNumSubscribers() == 0)
                && (position_pub.getNumSubscribers() == 0) && (marker_list_pub.getNumSubscribers() == 0)
                && (transform_grid_pub.getNumSubscribers() == 0) && (pose_grid_pub.getNumSubscribers() == 0)
                && (position_grid_pub.getNumSubscribers() == 0) && (pixel_pub.getNumSubscribers() == 0))
            {
                ROS_INFO("No subscribers, not looking for ArUco markers. Please, subscribe to start tracking.");
                return;
            }
            
            static tf::TransformBroadcaster br;

            if (cam_info_received)
            {
                ros::Time curr_stamp = msg->header.stamp;
                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                    // Copy ROS msg into OpenCV image format (inImage)
                    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
                    inImage = cv_ptr->image;
                
                    markers.clear();

                    // Detection of markers
                    //ROS_WARN("Starting detection");
                    mDetector.detect(inImage, markers, camParam, marker_size, false);

                    T.resize(markers.size());

                    markers_msg->markers.clear();
                    markers_msg->markers.resize(markers.size());
                    markers_msg->header.stamp = curr_stamp;
                    markers_msg->header.seq++;
                    
                    transforms_msg->transforms.clear();
                    transforms_msg->transforms.resize(markers.size());
                    transforms_msg->header.stamp = curr_stamp;

                    for (unsigned int i = 0; i < markers.size(); i++)
                    {

                        aruco_msgs::Marker & marker_i = markers_msg->markers.at(i);
                        marker_i.header.stamp = curr_stamp;
                        marker_i.id = markers.at(i).id;
                        marker_i.confidence = 1.0;

                        geometry_msgs::TransformStamped & transform_i = transforms_msg->transforms.at(i);

                        tf::StampedTransform cameraToReference;
                        cameraToReference.setIdentity();
                        getTransform(reference_frame, camera_frame, cameraToReference);

                        if (markers[i].id == marker0_id)// central marker
                        {
                            ROS_WARN("Central marker detected. Marker id: %d", marker0_id);
                            
                            tf::Transform Tm0 = aruco_ros::arucoMarker2Tf(markers[i]);

                            // Compute transformation wrt camera reference
                            Tm0 = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft) * Tm0;
                            T.at(i) = Tm0;
                            
                            // Broadcast transformation on /tf topic (marker to reference_frame)
                            tf::StampedTransform stampedTransform(Tm0, curr_stamp, reference_frame, marker0_frame);
                            br.sendTransform(stampedTransform);

                            // Create messages
                            tf::poseTFToMsg(Tm0, marker_i.pose.pose);
                            marker_i.header.frame_id = reference_frame;
                            
                            geometry_msgs::PoseStamped poseMsg;
                            tf::poseTFToMsg(Tm0, poseMsg.pose);
                            poseMsg.header.frame_id = reference_frame;
                            poseMsg.header.stamp = curr_stamp;
                            pose_pub.publish(poseMsg);

                            geometry_msgs::TransformStamped transformMsg;
                            tf::transformStampedTFToMsg(stampedTransform, transformMsg);
                            transform_pub.publish(transformMsg);                            

                            geometry_msgs::Vector3Stamped positionMsg;
                            positionMsg.header = transformMsg.header;
                            positionMsg.vector = transformMsg.transform.translation;
                            position_pub.publish(positionMsg);
                        }
                        else if (markers[i].id == marker1_id)
                        {
                            ROS_WARN("Pos1 marker detected. Marker id: %d", marker1_id);

                            // Marker1 transformation (reference_frame)
                            tf::Transform Tm1 = aruco_ros::arucoMarker2Tf(markers[i]);
                            Tm1 = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft) * Tm1;
                            
                            // Transformation of marker 1 to central tag
                            tf::Transform Tm1_lcs =  gridEstimator.Matrix2Tf(m1);
                            
                            // Effector pose using Marker1
                            tf::Transform Tm0_1 = Tm1 * Tm1_lcs.inverse();
                            T.at(i) = Tm0_1;

                            // Broadcast transformations on /tf topic (marker to reference_frame)
                            tf::StampedTransform stampedTransform1(Tm1, curr_stamp, reference_frame, marker1_frame);
                            br.sendTransform(stampedTransform1);
                            tf::StampedTransform stampedTransformMt1(Tm0_1, curr_stamp, reference_frame, "eff_refM1");
                            br.sendTransform(stampedTransformMt1);
                            
                            // Create messages
                            tf::transformStampedTFToMsg(stampedTransform1, transform_i);
                            transform_i.header.frame_id = reference_frame;
                            transform_i.child_frame_id = marker1_frame;
                            
                            tf::poseTFToMsg(Tm0_1, marker_i.pose.pose);
                            marker_i.header.frame_id = reference_frame;
                        }
                        else if (markers[i].id == marker2_id)
                        {
                            ROS_WARN("Pos2 marker detected. Marker id: %d", marker2_id);

                            // Marker2 transformation (reference_frame)
                            tf::Transform Tm2 = aruco_ros::arucoMarker2Tf(markers[i]);
                            Tm2 = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft) * Tm2;
                            
                            // Transformation of marker 1 to central tag
                            tf::Transform Tm2_lcs =  gridEstimator.Matrix2Tf(m2);
                            
                            // Effector pose using Marker1
                            tf::Transform Tm0_2 = Tm2 * Tm2_lcs.inverse();
                            T.at(i) = Tm0_2;

                            // Broadcast transformation on /tf topic (marker to reference_frame)
                            tf::StampedTransform stampedTransform2(Tm2, curr_stamp, reference_frame, marker2_frame);
                            br.sendTransform(stampedTransform2);
                            tf::StampedTransform stampedTransformMt2(Tm0_2, curr_stamp, reference_frame, "eff_refM2");
                            br.sendTransform(stampedTransformMt2);

                            // Create messages
                            tf::transformStampedTFToMsg(stampedTransform2, transform_i);
                            transform_i.header.frame_id = reference_frame;
                            transform_i.child_frame_id = marker2_frame;

                            tf::poseTFToMsg(Tm0_2, marker_i.pose.pose);
                            marker_i.header.frame_id = reference_frame;
                        }
                        else if (markers[i].id == marker3_id)
                        {
                            ROS_WARN("Pos3 marker detected. Marker id: %d", marker3_id);

                            // Marker 3 transformation (reference_frame)
                            tf::Transform Tm3 = aruco_ros::arucoMarker2Tf(markers[i]);
                            Tm3 = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft) * Tm3;
                            
                            // Transformation of marker 3 to central tag
                            tf::Transform Tm3_lcs =  gridEstimator.Matrix2Tf(m3);
                            
                            // Effector pose using marker 3
                            tf::Transform Tm0_3 = Tm3 * Tm3_lcs.inverse();
                            T.at(i) = Tm0_3;

                            // Broadcast transformation on /tf topic (marker to reference_frame)
                            tf::StampedTransform stampedTransform3(Tm3, curr_stamp, reference_frame, marker3_frame);
                            br.sendTransform(stampedTransform3);
                            tf::StampedTransform stampedTransformMt3(Tm0_3, curr_stamp, reference_frame, "eff_refM3");
                            br.sendTransform(stampedTransformMt3);

                            // Create messages
                            tf::transformStampedTFToMsg(stampedTransform3, transform_i);
                            transform_i.header.frame_id = reference_frame;
                            transform_i.child_frame_id = marker3_frame;
                            
                            tf::poseTFToMsg(Tm0_3, marker_i.pose.pose);
                            marker_i.header.frame_id = reference_frame;
                        }
                        else if (markers[i].id == marker4_id)
                        {
                            ROS_WARN("Pos4 marker detected. Marker id: %d", marker4_id);

                            // Marker 4 transformation (reference_frame)
                            tf::Transform Tm4 = aruco_ros::arucoMarker2Tf(markers[i]);
                            Tm4 = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft) * Tm4;
                            
                            // Transformation of marker 4 to central tag
                            tf::Transform Tm4_lcs =  gridEstimator.Matrix2Tf(m4);
                            
                            // Effector pose using marker 4
                            tf::Transform Tm0_4 = Tm4 * Tm4_lcs.inverse();
                            T.at(i) = Tm0_4;

                            // Broadcast transformation on /tf topic (marker to reference_frame)
                            tf::StampedTransform stampedTransform4(Tm4, curr_stamp, reference_frame, marker4_frame);
                            br.sendTransform(stampedTransform4);
                            tf::StampedTransform stampedTransformMt4(Tm0_4, curr_stamp, reference_frame, "eff_refM4");
                            br.sendTransform(stampedTransformMt4);

                            // Create messages
                            tf::transformStampedTFToMsg(stampedTransform4, transform_i);
                            transform_i.header.frame_id = reference_frame;
                            transform_i.child_frame_id = marker4_frame;

                            tf::poseTFToMsg(Tm0_4, marker_i.pose.pose);
                            marker_i.header.frame_id = reference_frame;
                        }
                    }

                    // Estimate pose effector                    
                    if (markers.size() > 0 && markers[0].id == marker0_id)
                    {
                        poseEstimation = gridEstimator.PoseEstimate(T);
                
                        // Publish effector pose
                        geometry_msgs::Vector3Stamped positionGridMsg;
                        geometry_msgs::TransformStamped transformGridMsg;
                        
                        tf::StampedTransform stampedGridTransform(poseEstimation.T, curr_stamp, reference_frame, "effector_frame");
                                                
                        tf::transformStampedTFToMsg(stampedGridTransform, transformGridMsg);
                        transform_grid_pub.publish(transformGridMsg);

                        geometry_msgs::PoseStamped poseGridMsg;
                        tf::poseTFToMsg(poseEstimation.T, poseGridMsg.pose);
                        poseGridMsg.header.frame_id = reference_frame;
                        poseGridMsg.header.stamp = curr_stamp;
                        pose_grid_pub.publish(poseGridMsg);

                        positionGridMsg.header = transformGridMsg.header;
                        positionGridMsg.vector = transformGridMsg.transform.translation;
                        position_grid_pub.publish(positionGridMsg);
                    }
                    
                    // publish markers' arrays and transforms
                    if (markers_msg->markers.size() > 0)
                    {
                        marker_pub.publish(markers_msg);
                        transforms_pub.publish(transforms_msg);

                    }
                    // Draw info and boundaries for each marker
                    for (std::size_t i = 0; i < markers.size(); i++)
                    {
                        markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
                    }
                
                    // Draw a 3d cube in each marker if there is 3d info
                    if (camParam.isValid() && marker_size != -1)
                    {
                        for (std::size_t i = 0; i < markers.size(); ++i)
                        {
                            aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
                        }
                    }                 
                
                    // Publish result image with augmented information
                    if (image_pub.getNumSubscribers() > 0)
                    {
                    cv_bridge::CvImage out_msg;
                    out_msg.header.stamp = curr_stamp;
                    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
                    out_msg.image = inImage;
                    image_pub.publish(out_msg.toImageMsg());
                    }

                    // Publish internal image results (threshold operation)
                    if (debug_pub.getNumSubscribers() > 0)
                    {
                    cv_bridge::CvImage debug_msg;
                    debug_msg.header.stamp = curr_stamp;
                    debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
                    debug_msg.image = mDetector.getThresholdedImage();
                    debug_pub.publish(debug_msg.toImageMsg());
                    }
                
                }// end try
                catch(cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
                }
            }// end if(cam_info_received)

        }

        // wait for one camerainfo, then shut down that subscriber
        void cam_info_callback(const sensor_msgs::CameraInfo &msg)
        {
            camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

            // handle cartesian offset between stereo pairs
            // see the sensor_msgs/CameraInfo documentation for details
            rightToLeft.setIdentity();
            rightToLeft.setOrigin(tf::Vector3(-msg.P[3] / msg.P[0], -msg.P[7] / msg.P[5], 0.0));

            ROS_WARN("camera info received!");
            cam_info_received = true;
            cam_info_sub.shutdown();
        }

        void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
        {
            mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
            normalizeImageIllumination = config.normalizeImage;
            dctComponentsToRemove = config.dctComponentsToRemove;
        }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "aruco_3dgrid");
    
    Aruco3D node;

    ros::spin();
}