#ifndef JPEG_STREAMERS_H_
#define JPEG_STREAMERS_H_

#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include "web_video_server/image_streamer.h"
#include "async_web_server_cpp/http_request.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "web_video_server/multipart_stream.h"

namespace web_video_server
{

class MjpegStreamer : public ImageTransportImageStreamer
{
public:
  MjpegStreamer(const async_web_server_cpp::HttpRequest &request, async_web_server_cpp::HttpConnectionPtr connection,
                ros::NodeHandle& nh);
  ~MjpegStreamer();
protected:
  virtual void sendImage(const cv::Mat &, const ros::Time &time);

private:
  MultipartStream stream_;
  int quality_;
};

class MjpegStreamerType : public ImageStreamerType
{
public:
  boost::shared_ptr<ImageStreamer> create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                   async_web_server_cpp::HttpConnectionPtr connection,
                                                   ros::NodeHandle& nh);
};

}

#endif
