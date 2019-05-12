// Headers in this package
#include "nnabla_vision_detection/nodelet.h"

// Headers in ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>

// Headers in STL
#include <fstream>
#include <map>
#include <memory>

// Headers in Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

// Headers in CUDA
#include <nbla_utils/nnp.hpp>
#ifdef WITH_CUDA
#include <nbla/cuda/cudnn/init.hpp>
#include <nbla/cuda/init.hpp>
#endif

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
                pnh_->param<std::string>("executor_name", executor_name_, "runtime");
                pnh_->param<std::string>("vision_info_topic", vision_info_topic_, "vision_info");
                pnh_->param<std::string>("nnp_file", nnp_file_, "");
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
                vision_info_pub_ = pnh_->advertise<vision_msgs::VisionInfo>(vision_info_topic_,1,true);
                vision_msgs::VisionInfo vision_info_msg;
                vision_info_msg.header.stamp = ros::Time::now();
                vision_info_msg.method = "nnabla_vision_detection";
                vision_info_msg.database_location = pnh_->getNamespace() + "/class_meta_info";
                vision_info_pub_.publish(vision_info_msg);
                result_pub_ = pnh_->advertise<vision_msgs::Detection2DArray>("result",1);
                initNnabla();
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
                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                } catch (cv_bridge::Exception &e)
                {
                    NODELET_ERROR("cv_bridge exception: %s", e.what());
                    return;
                }
                auto orig_size = cv_ptr->image.size();
                nbla::CgVariablePtr x = executor_->get_data_variables().at(0).variable;
                uint8_t *data = x->variable()->cast_data_and_get_pointer<uint8_t>(ctx_cpu_);
                if(x->variable()->ndim() != 4)
                {
                    NODELET_ERROR_STREAM("x variable dimension does not match, dimension should be 4, dim:" << x->variable()->ndim());
                    return;
                }
                auto inshape = x->variable()->shape();
                const int C = inshape[1];
                const int H = inshape[2];
                const int W = inshape[3];
                if(C != 3)
                {
                    NODELET_ERROR_STREAM("channel size does not match, channel size should be 3, channel size:" << C);
                    return;
                }
                cv::Mat resized;
                cv::resize(cv_ptr->image, resized, cv::Size{W, H});
                for (int hw = 0; hw < H * W; ++hw)
                {
                    for (int c = 0; c < C; ++c)
                    {
                        data[c * H * W + hw] = resized.ptr()[hw * C + 2 - c];
                    }
                }
                executor_->execute();
                nbla::CgVariablePtr y = executor_->get_output_variables().at(0).variable;
                const float *y_data = y->variable()->get_data_pointer<float>(ctx_cpu_);
                if(y->variable()->ndim() != 3)
                {
                    NODELET_ERROR_STREAM("y variable dimension does not match, dimension should be 3, dim:" << y->variable()->ndim());
                    return;
                }
                auto outshape = y->variable()->shape();
                int num_classes = (int)classes_.size();
                int num_c = 5 + num_classes;
                vision_msgs::Detection2DArray detection_array;
                detection_array.header = msg->header;
                for (int b = 0; b < outshape[1]; ++b)
                {
                    float score = -1;
                    int class_idx = 0;
                    for (int k = 0; k < num_classes; ++k)
                    {
                        const float score_k = y_data[b * num_c + 5 + k];
                        if (score_k > score)
                        {
                            class_idx = k;
                            score = score_k;
                        }
                    }
                    if (score <= 0)
                    {
                        continue;
                    }
                    const float x = y_data[b * num_c + 0];
                    const float y = y_data[b * num_c + 1];
                    const float w = y_data[b * num_c + 2];
                    const float h = y_data[b * num_c + 3];
                    const int x0 = transformCoord(x - w / 2, W);
                    const int y0 = transformCoord(y - h / 2, H);
                    const int x1 = transformCoord(x + w / 2, W);
                    const int y1 = transformCoord(y + h / 2, H);
                    std::string detected_class = classes_[class_idx];
                    vision_msgs::Detection2D detection;
                    detection.header = msg->header;
                    detection.bbox.center.x = ((float)x0 + (float)x1)/2.0;
                    detection.bbox.center.y = ((float)y0 + (float)y1)/2.0;
                    detection.bbox.size_x = std::fabs((float)x1 - (float)x0);
                    detection.bbox.size_y = std::fabs((float)y1 - (float)y0);
                    vision_msgs::ObjectHypothesisWithPose hypothesis;
                    hypothesis.score = score;
                    hypothesis.id = class_idx;
                    detection.results.push_back(hypothesis);
                    detection_array.detections.push_back(detection);
                }
                result_pub_.publish(detection_array);
                return;
            }

            void initNnabla()
            {
#ifdef WITH_CUDA
                try
                {
                    nbla::init_cudnn();
                }
                catch(...)
                {
                    NODELET_ERROR_STREAM("failed to initialize cudnn");
                    std::exit(-1);
                }
#endif
                try
                {
                    nnp_ptr_ = std::make_shared<nbla::utils::nnp::Nnp>(ctx_);
                    nnp_ptr_->add(nnp_file_);
                }
                catch(...)
                {
                    NODELET_ERROR_STREAM("failed to initialize nnabla utils");
                    std::exit(-1);
                }
                try
                {
                    executor_ = nnp_ptr_->get_executor(executor_name_);
                    executor_->set_batch_size(1);
                }
                catch(const std::exception& e)
                {
                    NODELET_ERROR_STREAM("failed to initialize nnabla executor");
                    std::exit(-1);
                }
                return;
            }

            inline int transformCoord(float x, int size)
            {
                return std::max(0, std::min((int)(x*size), size-1));
            }
        private:
            boost::shared_ptr<image_transport::ImageTransport> it_;
            image_transport::Subscriber img_sub_;
            ros::Publisher result_pub_;
            ros::Publisher vision_info_pub_;
            std::string image_topic_;
            std::string vision_info_topic_;
            std::string class_meta_file_;
            std::map<int,std::string> classes_;
            std::ifstream ifs_;
            std::shared_ptr<nbla::utils::nnp::Executor> executor_;
            std::string executor_name_;
            std::string nnp_file_;
            std::shared_ptr<nbla::utils::nnp::Nnp> nnp_ptr_;
            nbla::Context ctx_cpu_{{"cpu:float"}, "CpuCachedArray", "0"};
#ifdef WITH_CUDA
            nbla::Context ctx_{{"cudnn:float", "cuda:float", "cpu:float"}, "CudaCachedArray", "0"};
#else
            nbla::Context ctx_{{"cpu:float"}, "CpuCachedArray", "0"};
#endif
    };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nnabla_vision_detection::NnablaVisionDetectionNodelet, nodelet::Nodelet);