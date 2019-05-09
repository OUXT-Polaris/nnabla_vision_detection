#include <ros/ros.h>
#include "nnabla_vision_detection/nodelet.h"

namespace nnabla_vision_detection
{
    class NnablaVisionDetectionNodelet : public nnabla_vision_detection::Nodelet
    {
        public:
            virtual void onInit()  // NOLINT(modernize-use-override)
            {
                Nodelet::onInit();
                //it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));
                onInitPostProcess();
            }
            void subscribe()  // NOLINT(modernize-use-override)
            {
                NODELET_DEBUG("subscribe");
            }

            void unsubscribe()  // NOLINT(modernize-use-override)
            {
                NODELET_DEBUG("unsubscribe");
            }
    };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nnabla_vision_detection::NnablaVisionDetectionNodelet, nodelet::Nodelet);