#include "video_capture.h"
#include "spinnaker_camera_wrapper.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>

using namespace cv;
using namespace std;

Spinnaker::ImagePtr ImagePtrToColor(const Spinnaker::ImagePtr &imagePtr);
void ImagePtrToCvMat(const Spinnaker::ImagePtr &imagePtr, Mat &dst);

namespace toy
{
    class VideoCapture_Spinnaker : public VideoCapture
    {
    private:
        SystemPtr system;
        CameraList cam_list;
        shared_ptr<SpinnakerCameraWrapper> pscam;
        ImagePtr pimage;
        bool is_opened;
        int device_index;

    public:
        VideoCapture_Spinnaker(int index)
        {
            device_index = index;
            open(index);
            pscam->setBufferMode("NewestOnly");
        }
        virtual ~VideoCapture_Spinnaker()
        {
            release();
        }

        virtual bool open(int index)
        {
            // Retrieve singleton reference to system object
            system = System::GetInstance();

            // Retrieve list of cameras from the system
            cam_list = system->GetCameras();

            // Initialize camera
            pscam = make_shared<SpinnakerCameraWrapper>(cam_list.GetByIndex(index));
            pscam->init();

            if (!pscam->isStreaming())
                pscam->beginAcquisition();

            is_opened = true;
            return is_opened;
        }
        virtual void release()
        {
            if (pscam->isStreaming())
                pscam->endAcquisition();

            // Deinitialize camera
            pscam->deinit();

            // Clear camera list before releasing system
            // pscam == nullptr; // will not release resource for this position? optimization problem?
            pscam.reset();
            cam_list.Clear();

            // Release system
            system->ReleaseInstance();

            is_opened = false;
        }

        virtual bool grab()
        {
            if (!is_opened)
                return false;

            if (!pscam->isStreaming())
                pscam->beginAcquisition();

            int retry = 5;
            while (retry >= 0) {
                try // fix exception if grab after release;
                {
                    pimage = pscam->getNextImage(1000); // wait for at most 100ms[10fps]
                    break;
                }
                catch (const exception &e)
                {
                    SPDLOG_ERROR("Grab: {}, retry {}", e.what(), retry);
                    release();
                    usleep(100000);
                    open(device_index);
                    pscam->setBufferMode("NewestOnly");
                }
                retry = retry - 1;
            }

            return !pimage->IsIncomplete();
        }
        virtual bool retrieve(Mat &image, int flag)
        {
            ImagePtrToCvMat(ImagePtrToColor(pimage), image);
            pimage = nullptr;
            return true;
        }
        virtual bool read(Mat &image)
        {
            if (grab())
            {
                int flag;
                retrieve(image, flag);
                return true;
            }
            else
            {
                image = Mat();
                return false;
            }
        }

        virtual bool set(int propId, double value)
        {
            if (pscam->isStreaming())
                pscam->endAcquisition();

            if (propId == cv::CAP_PROP_FPS)
            {
                pscam->setBoolValue("AcquisitionFrameRateEnable", true);
                pscam->setFloatValue("AcquisitionFrameRate", value);
            }
            if (propId == cv::CAP_PROP_FRAME_WIDTH)
                pscam->setIntValue("Width", value);
            if (propId == cv::CAP_PROP_FRAME_HEIGHT)
                pscam->setIntValue("Height", value);
            return true;
        }
        virtual double get(int propId) const
        {
            if (propId == cv::CAP_PROP_FPS)
                return pscam->getFloatValue("AcquisitionFrameRate");
            if (propId == cv::CAP_PROP_FRAME_WIDTH)
                return pscam->getIntValue("Width");
            if (propId == cv::CAP_PROP_FRAME_HEIGHT)
                return pscam->getIntValue("Height");
            return -1;
        }
    };

    VideoCapture::Ptr createSpinnakerCapture(int index)
    {
        try
        {
            return make_shared<VideoCapture_Spinnaker>(index);
        }
        catch (exception &e)
        {
            SPDLOG_ERROR(e.what());
            return nullptr;
        }
    }

} // namespace toy

Spinnaker::ImagePtr ImagePtrToColor(const Spinnaker::ImagePtr &imagePtr)
{
    return imagePtr->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::DEFAULT);
}

/*
    * This function converts between Spinnaker::ImagePtr container to Mat container used in OpenCV.
*/
void ImagePtrToCvMat(const Spinnaker::ImagePtr &imagePtr, Mat &dst)
{
    try
    {
        const auto XPadding = imagePtr->GetXPadding();
        const auto YPadding = imagePtr->GetYPadding();
        const auto rowsize = imagePtr->GetWidth();
        const auto colsize = imagePtr->GetHeight();

        // Image data contains padding. When allocating Mat container size, you need to account for the X,Y
        // image data padding.
        Mat src =  Mat((int)(colsize + YPadding), (int)(rowsize + XPadding), CV_8UC3, imagePtr->GetData(),
                            imagePtr->GetStride());
        cvtColor(src, dst, COLOR_BGR2YUV_I420);
    }
    catch (const std::exception &e)
    {
        dst = Mat();
    }
}