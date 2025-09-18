/*
 Replicated from rocon_rtsp_camera_relay (BSD licensed)
 Vendored into rc_gimbal_bridge to provide the same functionality.
*/

#include <rc_gimbal_bridge/rtsp_camera_relay.hpp>
#include <sstream>
#include <cstdlib>

namespace rocon {

RoconRtspCameraRelay::RoconRtspCameraRelay(ros::NodeHandle &n) : nh_(n) {
  image_transport::ImageTransport it(nh_);
  pub_video_ = it.advertise("image", 1);
  pub_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
  pub_status_ = nh_.advertise<std_msgs::String>("status", 1);
}

RoconRtspCameraRelay::~RoconRtspCameraRelay() { vcap_.release(); }

bool RoconRtspCameraRelay::init(const std::string video_stream_url) {
  video_stream_address_ = video_stream_url;

  std::string backend_str;
  nh_.param<std::string>("backend", backend_str, std::string("any"));
  std::string rtsp_transport;
  nh_.param<std::string>("rtsp_transport", rtsp_transport, std::string(""));
  int timeout_ms = 0;
  nh_.param<int>("timeout_ms", timeout_ms, 0);
  // FFmpeg options
  bool ffmpeg_low_latency = true;
  nh_.param<bool>("ffmpeg_low_latency", ffmpeg_low_latency, true);
  std::string ffmpeg_capture_options;
  nh_.param<std::string>("ffmpeg_capture_options", ffmpeg_capture_options, std::string(""));

  // 保持 URL 原样，避免在路径后拼接查询串（部分 RTSP 服务器会因此返回 454）
  std::string open_url = video_stream_address_;

  // Build FFmpeg capture options string and set environment variable for OpenCV FFmpeg backend
  // Example: rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|framedrop;1|max_delay;0|reorder_queue_size;0
  {
    std::string opts;
    if (!ffmpeg_capture_options.empty()) {
      opts = ffmpeg_capture_options;
    } else {
      if (!rtsp_transport.empty()) {
        opts += (opts.empty() ? "" : "|");
        opts += std::string("rtsp_transport;") + rtsp_transport;
      }
      if (timeout_ms > 0) {
        std::ostringstream oss;
        oss << static_cast<long long>(timeout_ms) * 1000LL;
        opts += (opts.empty() ? "" : "|");
        opts += std::string("stimeout;") + oss.str();
      }
      if (ffmpeg_low_latency) {
        // Align with ffplay: -fflags nobuffer -flags low_delay -framedrop
        // Plus extra tweaks for low latency
        opts += (opts.empty() ? "" : "|");
        opts += "fflags;nobuffer|flags;low_delay|framedrop;1|max_delay;0|reorder_queue_size;0";
      }
    }
    if (!opts.empty()) {
      setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", opts.c_str(), 1);
      ROS_INFO("Rtsp Camera : OPENCV_FFMPEG_CAPTURE_OPTIONS='%s'", opts.c_str());
    }
  }

  ROS_INFO("Rtsp Camera : Opening %s (backend=%s)", open_url.c_str(), backend_str.c_str());

  // Prefer FFmpeg when backend is 'ffmpeg' or 'any' (to honour TCP options)
  if (backend_str == "ffmpeg" || backend_str == "any") {
    if (tryOpen(open_url, cv::CAP_FFMPEG)) return true;
  }
  // Try GStreamer if requested or as fallback
  if (backend_str == "gstreamer" || backend_str == "any") {
    ROS_WARN("Rtsp Camera : Fallback to GStreamer backend");
    if (tryOpen(open_url, cv::CAP_GSTREAMER)) return true;
  }
  // Last resort: CAP_ANY
  ROS_WARN("Rtsp Camera : Fallback to OpenCV CAP_ANY");
  if (tryOpen(open_url, cv::CAP_ANY)) return true;

  return false;
}

bool RoconRtspCameraRelay::reset(const std::string video_stream_url) {
  vcap_.release();
  return init(video_stream_url);
}

void RoconRtspCameraRelay::convertCvToRosImg(const cv::Mat &mat,
                                             sensor_msgs::Image &ros_img,
                                             sensor_msgs::CameraInfo &ci) {
  cv_bridge::CvImage cv_img;

  cv_img.encoding = sensor_msgs::image_encodings::BGR8;
  cv_img.image = mat;
  cv_img.toImageMsg(ros_img);
  ros_img.header.stamp = ros::Time::now();
  ci.header = ros_img.header;
  ci.width = ros_img.width;
  ci.height = ros_img.height;

  return;
}

void RoconRtspCameraRelay::spin() {
  cv::Mat mat;
  sensor_msgs::CameraInfo ci;
  sensor_msgs::Image ros_img;
  std_msgs::String ros_str;

  while (ros::ok()) {
    if (!vcap_.read(mat)) {
      status_ = "No frame from camera";
      // Avoid indefinite block when no frame
      cv::waitKey(1);
    } else {
      status_ = "live";
    }

    ros_str.data = status_;

    if (!mat.empty()) {
      convertCvToRosImg(mat, ros_img, ci);
      pub_video_.publish(ros_img);
      pub_camera_info_.publish(ci);
    }
    pub_status_.publish(ros_str);
    cv::waitKey(1);
  }
}

bool RoconRtspCameraRelay::tryOpen(const std::string &url, int api_pref) {
  bool ok = false;
  if (api_pref == cv::CAP_ANY) {
    ok = vcap_.open(url);
  } else {
    ok = vcap_.open(url, api_pref);
  }
  if (!ok) {
    ROS_ERROR("Rtsp Camera : Failed to open stream (api=%d)", api_pref);
  }
  return ok;
}

std::string RoconRtspCameraRelay::buildUrlWithQuery(
    const std::string &base,
    const std::vector<std::pair<std::string, std::string>> &params) {
  if (params.empty()) return base;
  std::ostringstream oss;
  oss << base;
  bool has_q = (base.find('?') != std::string::npos);
  oss << (has_q ? '&' : '?');
  for (size_t i = 0; i < params.size(); ++i) {
    if (i > 0) oss << '&';
    oss << params[i].first << '=' << params[i].second;
  }
  return oss.str();
}

} // namespace rocon
