#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <vector>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include "web_video_server/web_video_server.h"
#include "web_video_server/ros_compressed_streamer.h"
#include "web_video_server/jpeg_streamers.h"
#include "async_web_server_cpp/http_reply.hpp"

namespace web_video_server
{

static bool __verbose;

static std::string __default_stream_type;

static bool ros_connection_logger(async_web_server_cpp::HttpServerRequestHandler forward,
                                  const async_web_server_cpp::HttpRequest &request,
                                  async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                  const char* end)
{
  if (__verbose)
  {
    ROS_INFO_STREAM("Handling Request: " << request.uri);
  }
  try
  {
    forward(request, connection, begin, end);
    return true;
  }
  catch (std::exception &e)
  {
    ROS_WARN_STREAM("Error Handling Request: " << e.what());
    return false;
  }
  return false;
}

WebVideoServer::WebVideoServer(ros::NodeHandle &nh, ros::NodeHandle &private_nh) :
    nh_(nh), handler_group_(
        async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found))
{
  address_ = "0.0.0.0";
  port_ = 17777;
  __verbose = false;

  int server_threads = 2;
  ros_threads_ = 2;
  __default_stream_type = "mjpeg";

  stream_types_["mjpeg"] = boost::shared_ptr<ImageStreamerType>(new MjpegStreamerType());
  stream_types_["ros_compressed"] = boost::shared_ptr<ImageStreamerType>(new RosCompressedStreamerType());

  handler_group_.addHandlerForPath("/stream", boost::bind(&WebVideoServer::handle_stream, this, _1, _2, _3, _4));

  try
  {
    server_.reset(
        new async_web_server_cpp::HttpServer(address_, boost::lexical_cast<std::string>(port_),
                                             boost::bind(ros_connection_logger, handler_group_, _1, _2, _3, _4),
                                             server_threads));
  }
  catch(boost::exception& e)
  {
    ROS_ERROR("Exception when creating the web server! %s:%d", address_.c_str(), port_);
    throw;
  }
}

WebVideoServer::~WebVideoServer()
{
}

void WebVideoServer::spin()
{
  server_->run();
  ROS_INFO_STREAM("Waiting For connections on " << address_ << ":" << port_);

  ros::AsyncSpinner spinner(ros_threads_);
  spinner.start();

  while( ros::ok() ) {
    this->restreamFrames(0.5);
    usleep(100000);
  }

  server_->stop();
}

void WebVideoServer::restreamFrames( double max_age )
{
  boost::mutex::scoped_lock lock(subscriber_mutex_);
  for (auto &itr : image_subscribers_)
  {
    itr.second->restreamFrame( max_age );
  }
}

bool WebVideoServer::handle_stream(const async_web_server_cpp::HttpRequest &request,
                                   async_web_server_cpp::HttpConnectionPtr connection, const char* begin,
                                   const char* end)
{
  std::string type = request.get_query_param_value_or_default("type", __default_stream_type);
  std::string topic = request.get_query_param_value_or_default("topic", "");
  if (topic.find("/compressed") != std::string::npos) {
    type = "ros_compressed";
  }
  if (stream_types_.find(type) != stream_types_.end())
  {
    ROS_INFO_STREAM("create stream: " << type << ", image topic: " << topic);
    boost::shared_ptr<ImageStreamer> streamer = stream_types_[type]->create_streamer(request, connection, nh_);
    streamer->start();
    boost::mutex::scoped_lock lock(subscriber_mutex_);
    image_subscribers_[topic] = streamer;
  }
  else
  {
    async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)(request, connection, begin, end);
  }
  return true;
}

}
