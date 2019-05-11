// Headers in this package
#include "nnabla_vision_detection/nodelet.h"

// Headers in ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <vision_msgs/Classification2D.h>
#include <vision_msgs/VisionInfo.h>

// Headers in STL
#include <fstream>

// Headers in Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

namespace nnabla_vision_detection
{
    class NnablaVisionDetectionNodelet : public nnabla_vision_detection::Nodelet
    {
        public:
            virtual void onInit()  // NOLINT(modernize-use-override)
            {
                Nodelet::onInit();
                it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));
                pnh_->param<std::string>("image_topic", image_topic_, "image_raw");
                pnh_->param<std::string>("class_meta_file", class_meta_file_, "");
                //vision_info_pub_ = pnh_.advertise();
                onInitPostProcess();
                return;
            }

            void subscribe()  // NOLINT(modernize-use-override)
            {
                NODELET_DEBUG("subscribe");
                img_sub_ = it_->subscribe(image_topic_, 1, &NnablaVisionDetectionNodelet::imageCallback, this);
                return;
            }

            void unsubscribe()  // NOLINT(modernize-use-override)
            {
                NODELET_DEBUG("unsubscribe");
                img_sub_.shutdown();
                return;
            }
            
            void imageCallback(const sensor_msgs::ImageConstPtr& msg)
            {
                return;
            }
        private:
            boost::shared_ptr<image_transport::ImageTransport> it_;
            image_transport::Subscriber img_sub_;
            ros::Publisher result_pub_;
            ros::Subscriber vision_info_pub_;
            std::string image_topic_;
            std::string class_meta_file_;
            std::vector<std::string> classes_;
    };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nnabla_vision_detection::NnablaVisionDetectionNodelet, nodelet::Nodelet);