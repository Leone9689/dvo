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

#include <dvo_benchmark/camera_base.h>


CameraBase::CameraBase(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  nh_(nh),
  nh_private_(nh_private),

  /*rgb_image_subscriber_(nh, "camera/rgb/image_color", 1),
  depth_image_subscriber_(nh, "camera/depth/image", 1),

  rgb_camera_info_subscriber_(nh, "camera/rgb/camera_info", 1),
  depth_camera_info_subscriber_(nh, "camera/depth/camera_info", 1),
  */

  //synchronizer_(RGBDWithCameraInfoPolicy(5), rgb_image_subscriber_, depth_image_subscriber_, rgb_camera_info_subscriber_, depth_camera_info_subscriber_),

  connected(false)
{
  filename ="/home/leone/Documents/bechmark/rgbd_dataset_freiburg1_desk.bag";
  
  visua_tpc  = "/camera/rgb/image_color";                                                           
  depth_tpc  = "/camera/depth/image";
  visua_info_tpc  = "/camera/rgb/camera_info";
  depth_info_tpc  = "/camera/depth/camera_info";
  
  loadBagFakeSubscriberSetup(visua_tpc, depth_tpc, visua_info_tpc,depth_info_tpc);
}

void CameraBase::loadBagFakeSubscriberSetup(const std::string& visua_tpc,
                                            const std::string& depth_tpc,
                                            const std::string& visua_info_tpc,
                                            const std::string& depth_info_tpc)
{
    depth_img_sub_ = new BagSubscriber<sensor_msgs::Image>();
    rgb_img_sub_ = new BagSubscriber<sensor_msgs::Image>();
    rgb_info_sub_ = new BagSubscriber<sensor_msgs::CameraInfo>();
    depth_info_sub_ = new BagSubscriber<sensor_msgs::CameraInfo>();
    
    synchronizer_ = new message_filters::Synchronizer<RGBDWithCameraInfoPolicy>(RGBDWithCameraInfoPolicy(4),  *rgb_img_sub_, *depth_img_sub_,*rgb_info_sub_,*depth_info_sub_);
    //ROS_INFO_STREAM_NAMED("OpenNIListener", "Listening to " << visua_tpc << ", " << depth_tpc << " and " << cinfo_tpc);
    
}

bool asyncFrameDrop(ros::Time depth, ros::Time rgb)                                                   
{            
  long rgb_timediff = abs(static_cast<long>(rgb.nsec) - static_cast<long>(depth.nsec));
  if(rgb_timediff > 33333333)
  {
    ROS_DEBUG("Depth image time: %d - %d", depth.sec,   depth.nsec);
    ROS_DEBUG("RGB   image time: %d - %d", rgb.sec, rgb.nsec);
    ROS_INFO("Depth and RGB image off more than 1/30sec: %li (nsec)", rgb_timediff);
    return true;
           
  } 
  else 
  {   
    ROS_DEBUG("Depth image time: %d - %d", depth.sec,   depth.nsec);
    ROS_DEBUG("RGB   image time: %d - %d", rgb.sec, rgb.nsec);
  }          
  return false;
}            

void CameraBase::processBagfile(std::string filename,
                                const std::string& visua_tpc,
                                const std::string& depth_tpc,
                                const std::string& visua_info_tpc,
                                const std::string& depth_info_tpc)
{
 
  ROS_INFO("PROCESSING");
  ROS_INFO_NAMED("OpenNIListener", "Loading Bagfile %s", filename.c_str());
  { //bag will be destructed after this block (hopefully frees memory for the optimizer)
    rosbag::Bag input_bag;
    try{
      input_bag.open(filename, rosbag::bagmode::Read);
    } catch(rosbag::BagIOException ex) {
      ROS_FATAL("Opening Bagfile %s failed: %s Quitting!", filename.c_str(), ex.what());
      ros::shutdown();
      return;
    }
    ROS_INFO_NAMED("OpenNIListener", "Opened Bagfile %s", filename.c_str());

    std::vector<std::string> topics;
    topics.push_back(visua_tpc);
    topics.push_back(depth_tpc);
    topics.push_back(visua_info_tpc);
    topics.push_back(depth_info_tpc);

    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    // Simulate sending of the messages in the bagfile
    std::deque<sensor_msgs::Image::ConstPtr> vis_images;
    std::deque<sensor_msgs::Image::ConstPtr> dep_images;
    std::deque<sensor_msgs::CameraInfo::ConstPtr> vis_infos;
    std::deque<sensor_msgs::CameraInfo::ConstPtr> dep_infos;
    
   // ros::Time last_tf=ros::TIME_MIN;

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      usleep(10);
      if(!ros::ok()) return;
      
      //ROS_INFO_NAMED("OpenNIListener", "Processing %s of type %s with timestamp %f", m.getTopic().c_str(), m.getDataType().c_str(), m.getTime().toSec());

      if (m.getTopic() == visua_tpc || ("/" + m.getTopic() == visua_tpc))
      {
        sensor_msgs::Image::ConstPtr rgb_img = m.instantiate<sensor_msgs::Image>();
        if (rgb_img !=NULL)  rgb_img_sub_->newMessage(rgb_img);
        
      }
      else if (m.getTopic() == depth_tpc || ("/" + m.getTopic() == depth_tpc))
      {
        sensor_msgs::Image::ConstPtr depth_img = m.instantiate<sensor_msgs::Image>();
        if (depth_img !=NULL)  depth_img_sub_->newMessage(depth_img);
      }

      else if (m.getTopic() == visua_info_tpc || ("/" + m.getTopic() == visua_info_tpc))
      {
        sensor_msgs::CameraInfo::ConstPtr cam_info = m.instantiate<sensor_msgs::CameraInfo>();
        if (cam_info !=NULL) rgb_info_sub_->newMessage(cam_info);
      }  
      else if (m.getTopic() == depth_info_tpc || ("/" + m.getTopic() == depth_info_tpc))
      {
        sensor_msgs::CameraInfo::ConstPtr cam_info = m.instantiate<sensor_msgs::CameraInfo>();
        if (cam_info !=NULL) depth_info_sub_->newMessage(cam_info);
      }  


    }
    ROS_WARN_NAMED("eval", "Finished processing of Bagfile");
    input_bag.close();
  }
}



CameraBase::~CameraBase()
{
  stopSynchronizedImageStream();
}

bool CameraBase::isSynchronizedImageStreamRunning()
{
  return connected;
}

void CameraBase::startSynchronizedImageStream()
{
  if(!connected)
  {
    //connection = synchronizer_.registerCallback(boost::bind(&CameraBase::handleImages, this, _1, _2, _3, _4));
    
    connection = synchronizer_->registerCallback(boost::bind(&CameraBase::handleImages, this, _1, _2, _3,_4));
    processBagfile(filename, visua_tpc, depth_tpc, visua_info_tpc,depth_info_tpc);
    
    connected = true;
  }
}

void CameraBase::stopSynchronizedImageStream()
{
  if(connected)
  {
    connection.disconnect();
    connected = false;
  }
}

