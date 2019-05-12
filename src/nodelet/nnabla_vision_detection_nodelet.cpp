// Headers in this package
#include "nnabla_vision_detection/nodelet.h"

// Headers in ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <vision_msgs/Classification2D.h>
#include <vision_msgs/VisionInfo.h>

// Headers in STL
#include <fstream>
#include <map>

// Headers in Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

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
                ifs_.open(class_meta_file_);
                if (ifs_.fail())
                {
                    NODELET_ERROR_STREAM("failed to read vision_info, file " << class_meta_file_ << " does not exist.");
                    std::exit(-1);
                }
                std::string class_meta_str = "";
                std::string str;
                while (getline(ifs_, str))
                {
                    class_meta_str = class_meta_str+str+"\n";
                }
                ifs_.close();
                pnh_->setParam("class_meta_info", class_meta_str);
                using namespace boost::property_tree;
                ptree pt;
                read_xml(class_meta_file_, pt);
                BOOST_FOREACH (const ptree::value_type& child, pt.get_child("vision_info"))
                {
                    if(child.first == "class")
                    {
                        boost::optional<int> id = child.second.get_optional<int>("<xmlattr>.id");
                        boost::optional<std::string> name = child.second.get_optional<std::string>("<xmlattr>.name");
                        if(id && name)
                        {
                            classes_[*id] = *name;
                        }
                        else
                        {
                            NODELET_ERROR_STREAM("failed to read xml string, file " << class_meta_file_ << " does not exist.");
                            std::exit(-1);
                        }
                    }
                }
                vision_info_pub_ = pnh_->advertise<vision_msgs::VisionInfo>("vision_info",1,true);
                vision_msgs::VisionInfo vision_info_msg;
                vision_info_msg.header.stamp = ros::Time::now();
                vision_info_msg.method = "nnabla_vision_detection";
                vision_info_msg.database_location = pnh_->getNamespace() + "/class_meta_info";
                vision_info_pub_.publish(vision_info_msg);
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
            ros::Publisher vision_info_pub_;
            std::string image_topic_;
            std::string class_meta_file_;
            std::map<int,std::string> classes_;
            std::ifstream ifs_;
    };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nnabla_vision_detection::NnablaVisionDetectionNodelet, nodelet::Nodelet);