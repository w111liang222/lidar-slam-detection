#ifndef MULTIPART_STREAM_H_
#define MULTIPART_STREAM_H_

#include <ros/ros.h>
#include <async_web_server_cpp/http_connection.hpp>

#include <queue>

namespace web_video_server
{

struct PendingFooter {
  ros::Time timestamp;
  boost::weak_ptr<std::string> contents;
};

class MultipartStream {
public:
  MultipartStream(async_web_server_cpp::HttpConnectionPtr& connection,
                  const std::string& boundry="boundarydonotcross",
                  std::size_t max_queue_size=3);

  void sendInitialHeader();
  void sendPartHeader(const ros::Time &time, const std::string& type, size_t payload_size);
  void sendPartFooter(const ros::Time &time);
  void sendPartAndClear(const ros::Time &time, const std::string& type, std::vector<unsigned char> &data);
  void sendPart(const ros::Time &time, const std::string& type, const boost::asio::const_buffer &buffer,
		async_web_server_cpp::HttpConnection::ResourcePtr resource);

private:
  bool isBusy();

private:
  const std::size_t max_queue_size_;
  async_web_server_cpp::HttpConnectionPtr connection_;
  std::string boundry_;
  std::queue<PendingFooter> pending_footers_;
};

}

#endif
