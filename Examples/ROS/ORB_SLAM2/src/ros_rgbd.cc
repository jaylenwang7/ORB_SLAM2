/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<algorithm>
#include<fstream>
#include<chrono>
#include<math.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "geometry_msgs/TransformStamped.h"
#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "../../../include/Converter.h"


using namespace std;
using namespace tf;
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

tf::Quaternion hamiltonProduct(tf::Quaternion a, tf::Quaternion b) {

	tf::Quaternion c;

		c[0] = (a[0]*b[0]) - (a[1]*b[1]) - (a[2]*b[2]) - (a[3]*b[3]);
		c[1] = (a[0]*b[1]) + (a[1]*b[0]) + (a[2]*b[3]) - (a[3]*b[2]);
		c[2] = (a[0]*b[2]) - (a[1]*b[3]) + (a[2]*b[0]) + (a[3]*b[1]);
		c[3] = (a[0]*b[3]) + (a[1]*b[2]) - (a[2]*b[1]) + (a[3]*b[0]);

	return c;
}

geometry_msgs::Vector3 SetVector3(float x, float y, float z){
	geometry_msgs::Vector3 Vec;
	Vec.x = x;
	Vec.y = y;
	Vec.z = z;
	return Vec;
}

geometry_msgs::Quaternion setQuat(float qx, float qy, float qz, float qw){
	geometry_msgs::Quaternion q;
	q.x = qx;
	q.y = qy;
	q.z = qz;
	q.w = qw;
	return q;
}

geometry_msgs::Quaternion quatProd(geometry_msgs::Quaternion q1,
	                               geometry_msgs::Quaternion q2){
	geometry_msgs::Quaternion qout;
	Eigen::Vector3d q1_v, q2_v, qout_v;
	double q1_s, q2_s, qout_s;

	//Vector parts of the quaternions
	q1_v << q1.x, q1.y, q1.z;
	q2_v << q2.x, q2.y, q2.z;

	//Scalar parts of the quaternions
	q1_s = q1.w;
	q2_s = q2.w;

	//Quaternion multiplication formula
	qout_v = q1_s*q2_v + q2_s*q1_v - q1_v.cross(q2_v);
	qout_s = q1_s*q2_s - q1_v.dot(q2_v);

	//Assign to quaternion structure
	qout = setQuat(qout_v(0), qout_v(1), qout_v(2), qout_s);
	return qout;
}

geometry_msgs::Vector3 quat2rpy(geometry_msgs::Quaternion quat){
	double qx, qy, qz, qw, roll, pitch, yaw;
	qx = quat.x;
	qy = quat.y;
	qz = quat.z;
	qw = quat.w;

	//Formulas for roll, pitch, yaw
	roll = atan2(2*(qw*qx + qy*qz) , 1 - 2*(qx*qx + qy*qy) );
	pitch = asin(2*(qw*qy - qz*qx));
	yaw = atan2(2*(qw*qz + qx*qy),1 - 2*(qy*qy + qz*qz) );

	geometry_msgs::Vector3 rpy = SetVector3(roll, pitch, yaw);
	return rpy;
}

geometry_msgs::Quaternion rpy2quat(geometry_msgs::Vector3 rpy){
	double roll, pitch, yaw;
	geometry_msgs::Quaternion q1, q2, q3, qout;

	roll = rpy.x;
	pitch = rpy.y;
	yaw = rpy.z;

	q1 = setQuat(sin(roll/2), 0, 0, cos(roll/2));
	q2 = setQuat(0, sin(pitch/2), 0, cos(pitch/2));
	q3 = setQuat(0, 0, sin(yaw/2), cos(yaw/2));

	qout = quatProd(q1, quatProd(q2, q3));

	return qout;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if (Tcw.empty())
		return;

	geometry_msgs::Pose pose;

	cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

	tf::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);

    static tf::TransformBroadcaster br_orb;

	tf::Transform transformCamera_orb;
	transformCamera_orb.setRotation(tf_quaternion);

	// found on airsim img publisher
    transformCamera_orb.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

	tf::poseTFToMsg(transformCamera_orb, pose);

	transformCamera_orb.setOrigin(tf::Vector3(pose.position.x,
                                        pose.position.z,
                                        -pose.position.y));

    geometry_msgs::Vector3 rpy_orb =  quat2rpy(pose.orientation);
    rpy_orb.y = -rpy_orb.y;
    rpy_orb.z = -rpy_orb.z + M_PI/2.0;

    geometry_msgs::Quaternion q_body2cam_orb = setQuat(0.5, -0.5, 0.5, -0.5);

    geometry_msgs::Quaternion q_cam_orb = rpy2quat(rpy_orb);
    q_cam_orb = quatProd(q_body2cam_orb, q_cam_orb);
    transformCamera_orb.setRotation(tf::Quaternion(q_cam_orb.x,
                                             q_cam_orb.y,
                                             q_cam_orb.z, 
                                             q_cam_orb.w));
    br_orb.sendTransform(tf::StampedTransform(transformCamera_orb, ros::Time::now(), "world", "orb_slam2_rgbd"));

	/*
    tf::Transform cam2quad(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.35));
    // cam2quad.setOrigin(tf::Vector3(0, -0.45, 0));
    br_orb.sendTransform(tf::StampedTransform(cam2quad, ros::Time::now(), "camera_orb", "orb_slam2_rgbd"));
	*/
	
	
	/*
    //Quaternion
    tf::Matrix3x3 tf3d;
    tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
	    pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
	    pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);
    double aux = tfqt[0];
	    tfqt[0]=-tfqt[2];
	    tfqt[2]=tfqt[1];
	    tfqt[1]=aux;

    //Translation for camera
    tf::Vector3 origin;
    origin.setValue(pose.at<float>(0,3),pose.at<float>(1,3),pose.at<float>(2,3));
    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
					    					0, 0, 1,
					    					-1, 0, 0);

    tf::Vector3 translationForCamera = origin * rotation270degXZ;

    //Hamilton (Translation for world)
    tf::Quaternion quaternionForHamilton(tfqt[3], tfqt[0], tfqt[1], tfqt[2]);
    tf::Quaternion secondQuaternionForHamilton(tfqt[3], -tfqt[0], -tfqt[1], -tfqt[2]);
    tf::Quaternion translationHamilton(0, translationForCamera[0], translationForCamera[1], translationForCamera[2]);

    tf::Quaternion translationStepQuat;
    translationStepQuat = hamiltonProduct(hamiltonProduct(quaternionForHamilton, translationHamilton), secondQuaternionForHamilton);

    tf::Vector3 translation(translationStepQuat[1], translationStepQuat[2], translationStepQuat[3]);

    //Creates transform and populates it with translation and quaternion
    tf::Transform transformCurrent;
    transformCurrent.setOrigin(translation);
    transformCurrent.setRotation(tfqt);
    
    //Publishes transform
    static tf::TransformBroadcaster br;

    br.sendTransform(tf::StampedTransform(transformCurrent, ros::Time::now(), "world", "orb_slam2_rgbd"));
	*/
}


