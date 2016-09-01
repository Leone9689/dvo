/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CAMERA_BASE_H_
#define CAMERA_BASE_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rosbag/bag.h>                                     
//For rosbag reading                                        
#include <rosbag/view.h>                                    
#include <boost/foreach.hpp>  

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>         
{   
  public:
  void newMessage(const boost::shared_ptr<M const> &msg) 
  {
    this->signalMessage(msg); //"this->" is required as of ros groovy
  } 
};  


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,                         
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo,
                                                        sensor_msgs::CameraInfo
                                                        > RGBDWithCameraInfoPolicy;
class CameraBase
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  message_filters::Subscriber<sensor_msgs::Image> rgb_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> depth_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_camera_info_subscriber_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_subscriber_;

  message_filters::Synchronizer<RGBDWithCameraInfoPolicy>* synchronizer_;
  //message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  std::string filename;  
  std::string visua_tpc;
  std::string depth_tpc;
  std::string visua_info_tpc;
  std::string depth_info_tpc;

  BagSubscriber<sensor_msgs::Image>* rgb_img_sub_;      
  BagSubscriber<sensor_msgs::Image>* depth_img_sub_;
  BagSubscriber<sensor_msgs::CameraInfo>* rgb_info_sub_;
  BagSubscriber<sensor_msgs::CameraInfo>* depth_info_sub_;



  bool isSynchronizedImageStreamRunning();

  void startSynchronizedImageStream();
  void stopSynchronizedImageStream();
public:
  CameraBase(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  
  //void loadBagFile(std::string filename);   
  void loadBagFakeSubscriberSetup(const std::string& visua_tpc,
                                  const std::string& depth_tpc,
                                  const std::string& visua_info_tpc,
                                  const std::string& depth_info_tpc);
    
  /*void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,   
                        const sensor_msgs::ImageConstPtr& depth_img_msg,    
                        const sensor_msgs::CameraInfoConstPtr& cam_info_msg);
  */  
  void processBagfile(std::string filename,
                      const std::string& visua_tpc,
                      const std::string& depth_tpc,
                      const std::string& visua_info_tpc,
                      const std::string& depth_info_tpc);

  
  virtual ~CameraBase();

  virtual void handleImages(
      const sensor_msgs::Image::ConstPtr& rgb_image_msg,
      const sensor_msgs::Image::ConstPtr& depth_image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
      const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg
  ) = 0;
private:
  message_filters::Connection connection;
  bool connected;
};

#endif /* CAMERA_BASE_H_ */
