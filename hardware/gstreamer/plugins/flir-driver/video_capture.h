// opencv video capture like interface
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>

namespace toy
{
    class VideoCapture
    {
    public:
        using Ptr = std::shared_ptr<VideoCapture>;

    public:
        // VideoCapture(int index);
        virtual ~VideoCapture() = default;

        virtual bool open(int index) = 0;
        virtual void release() = 0;

        virtual bool grab() = 0;
        virtual bool retrieve(cv::Mat &image, int flag) = 0;
        virtual bool read(cv::Mat &image) = 0;

        virtual bool set(int, double) = 0;
        virtual double get(int) const = 0;
    };

    VideoCapture::Ptr createSpinnakerCapture(int index = 0);
} // namespace toy