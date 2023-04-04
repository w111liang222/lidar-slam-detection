#include "web_video_server/jpeg_streamers.h"
#include "async_web_server_cpp/http_reply.hpp"

namespace web_video_server
{

MjpegStreamer::MjpegStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageTransportImageStreamer(request, connection, nh), stream_(connection)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 80);
  stream_.sendInitialHeader();
}

MjpegStreamer::~MjpegStreamer()
{
  this->inactive_ = true;
  boost::mutex::scoped_lock lock(send_mutex_); // protects sendImage.
}

void MjpegStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{
  std::vector<int> encode_params;
  encode_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", img, encoded_buffer, encode_params);

  stream_.sendPartAndClear(time, "image/jpeg", encoded_buffer);
}

boost::shared_ptr<ImageStreamer> MjpegStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                                    async_web_server_cpp::HttpConnectionPtr connection,
                                                                    ros::NodeHandle& nh)
{
  return boost::shared_ptr<ImageStreamer>(new MjpegStreamer(request, connection, nh));
}

}