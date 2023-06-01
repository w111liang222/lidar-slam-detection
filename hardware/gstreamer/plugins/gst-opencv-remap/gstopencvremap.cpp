#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/base/base.h>
#include <gst/controller/controller.h>
#include <gst/video/video.h>

#include "gstopencvremap.h"

GST_DEBUG_CATEGORY_STATIC (gst_opencv_remap_debug);
#define GST_CAT_DEFAULT gst_opencv_remap_debug

static void undistortion( cv::Mat& image );
gboolean gst_opencv_remap_sink_event (GstBaseTransform * base, GstEvent * event);

/* Filter signals and args */
enum
{
    /* FILL ME */
    LAST_SIGNAL
};

enum
{
    PROP_0
};

static GstStaticPadTemplate sink_template =
        GST_STATIC_PAD_TEMPLATE (
            "sink",
            GST_PAD_SINK,
            GST_PAD_ALWAYS,
            GST_STATIC_CAPS(
                ("video/x-raw, "
                 "format = (string)I420, "
                 "width = (int) [ 1, 32768 ], "
                 "height = (int) [ 1, 32768 ] , "
                 "framerate = (fraction) [ 0/1, MAX ]")  ) );

static GstStaticPadTemplate src_template =
        GST_STATIC_PAD_TEMPLATE (
            "src",
            GST_PAD_SRC,
            GST_PAD_ALWAYS,
            GST_STATIC_CAPS(
                ("video/x-raw, "
                 "format = (string)I420, "
                 "width = (int) [ 1, 32768 ], "
                 "height = (int) [ 1, 32768 ] , "
                 "framerate = (fraction) [ 0/1, MAX ]")  ) );

#define gst_opencv_remap_parent_class parent_class
G_DEFINE_TYPE (GstOpencvRemap, gst_opencv_remap, GST_TYPE_BASE_TRANSFORM);

static void gst_opencv_remap_set_property (GObject * object, guint prop_id,
                                             const GValue * value, GParamSpec * pspec);
static void gst_opencv_remap_get_property (GObject * object, guint prop_id,
                                             GValue * value, GParamSpec * pspec);

static GstFlowReturn gst_opencv_remap_transform_ip (GstBaseTransform * base,
                                                      GstBuffer * outbuf);

/* GObject vmethod implementations */

/* initialize the plugin's class */
static void
gst_opencv_remap_class_init (GstOpencvRemapClass * klass)
{
    GObjectClass *gobject_class;
    GstElementClass *gstelement_class;

    gobject_class = (GObjectClass *) klass;
    gstelement_class = (GstElementClass *) klass;

    gobject_class->set_property = gst_opencv_remap_set_property;
    gobject_class->get_property = gst_opencv_remap_get_property;

    gst_element_class_set_details_simple (gstelement_class,
                                          "OpencvRemap",
                                          "Generic/Filter",
                                          "OpenCV Remap",
                                          "LiangWang <15lwang@alumni.tongji.edu.cn>");

    gst_element_class_add_pad_template (gstelement_class,
                                        gst_static_pad_template_get (&src_template));
    gst_element_class_add_pad_template (gstelement_class,
                                        gst_static_pad_template_get (&sink_template));

    GST_BASE_TRANSFORM_CLASS (klass)->transform_ip =
            GST_DEBUG_FUNCPTR (gst_opencv_remap_transform_ip);

    GST_BASE_TRANSFORM_CLASS (klass)->sink_event = GST_DEBUG_FUNCPTR (gst_opencv_remap_sink_event);

    //debug category for filtering log messages
    GST_DEBUG_CATEGORY_INIT (gst_opencv_remap_debug, "opencvremap", 0, "Opencv Remap");
}

/* initialize the new element
 * initialize instance structure
 */
static void
gst_opencv_remap_init (GstOpencvRemap *filter)
{
    filter->img_w = 0;
    filter->img_h = 0;
}

static void
gst_opencv_remap_set_property (GObject * object, guint prop_id,
                                 const GValue * value, GParamSpec * pspec)
{
    GstOpencvRemap *filter = GST_OPENCV_REMAP (object);

    switch (prop_id) {
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
        break;
    }
}

static void
gst_opencv_remap_get_property (GObject * object, guint prop_id,
                                 GValue * value, GParamSpec * pspec)
{
    GstOpencvRemap *filter = GST_OPENCV_REMAP (object);

    switch (prop_id) {
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
        break;
    }
}

/* GstBaseTransform vmethod implementations */
gboolean gst_opencv_remap_sink_event (GstBaseTransform * base, GstEvent * event)
{
    GstEventType type;
    GstOpencvRemap *filter = GST_OPENCV_REMAP (base);

    type = GST_EVENT_TYPE (event);

    GST_TRACE_OBJECT( filter, "Event %d [%d]", type, GST_EVENT_CAPS  );

    switch (type) {
    case GST_EVENT_EOS:
        break;
    case GST_EVENT_CAPS:
    {
        GST_DEBUG_OBJECT( filter, "Event CAPS" );
        GstCaps* caps;

        gst_event_parse_caps(event, &caps);

        GstVideoInfo vinfo_in;
        gst_video_info_from_caps(&vinfo_in, caps);
        filter->img_w = vinfo_in.width;
        filter->img_h = vinfo_in.height;
        break;
    }
    default:
        break;
    }

    return GST_BASE_TRANSFORM_CLASS(parent_class)->sink_event(base, event);
}

/* this function does the actual processing
 */
static GstFlowReturn
gst_opencv_remap_transform_ip (GstBaseTransform * base, GstBuffer * outbuf)
{
    GstOpencvRemap *filter = GST_OPENCV_REMAP (base);
    GST_TRACE_OBJECT( filter, "transform_ip" );

    GstMapInfo map_buf;

    if (GST_CLOCK_TIME_IS_VALID (GST_BUFFER_TIMESTAMP (outbuf)))
        gst_object_sync_values (GST_OBJECT (filter), GST_BUFFER_TIMESTAMP (outbuf));

    if(FALSE==gst_buffer_map(outbuf, &map_buf, GstMapFlags(GST_MAP_READ|GST_MAP_WRITE)))
    {
        GST_WARNING_OBJECT( filter, "Could not map buffer for write/read" );
        return GST_FLOW_OK;
    }


    cv::Mat im = cv::Mat( filter->img_h * 1.5, filter->img_w, CV_8UC1, map_buf.data );

    GST_TRACE_OBJECT( filter, "Filter frame Size: %d x %d", filter->img_w, filter->img_h );

    // Draw 2D detections
    undistortion( im );

    GST_TRACE ("Buffer unmap" );
    gst_buffer_unmap( outbuf, &map_buf );

    return GST_FLOW_OK;
}


/* entry point to initialize the plug-in
         * initialize the plug-in itself
         * register the element factories and other features
         */
static gboolean
plugin_init (GstPlugin * plugin)
{
    return gst_element_register (plugin, "opencvremap", GST_RANK_NONE,
                                 GST_TYPE_OPENCV_REMAP);
}

GST_PLUGIN_DEFINE (
        GST_VERSION_MAJOR,
        GST_VERSION_MINOR,
        opencvremap,
        "OPENCV REMAP",
        plugin_init,
        GST_PACKAGE_VERSION,
        GST_PACKAGE_LICENSE,
        GST_PACKAGE_NAME,
        GST_PACKAGE_ORIGIN
        )


#define CONFIG_FILE_PATH "/tmp/camera_config"

typedef struct {
  cv::Mat xmap;
  cv::Mat ymap;

  bool is_wrap;
  cv::Size out_size;
  cv::Size perspective_size;
  cv::Mat homography;
  cv::Mat affine;
} Config_t;

static std::map<pid_t, std::string> name_map;
static std::map<std::string, Config_t> config_map;

std::vector<std::string> get_config() {
  std::ifstream f(CONFIG_FILE_PATH);
  std::string s;
  std::vector<std::string> result;
  while (getline(f, s, ' ')) {
      result.push_back(s);
  }
  std::remove(CONFIG_FILE_PATH);
  return result;
}

void get_cv_remap(std::vector<std::string> config, cv::Mat &xmap, cv::Mat &ymap, cv::Size &output_size) {
  int   w  = std::stoi(config[1]);
  int   h  = std::stoi(config[2]);
  float fx = std::stof(config[3]);
  float fy = std::stof(config[4]);
  float cx = std::stof(config[5]);
  float cy = std::stof(config[6]);
  float k1 = std::stof(config[7]);
  float k2 = std::stof(config[8]);
  float p1 = std::stof(config[9]);
  float p2 = std::stof(config[10]);
  int fisheye = std::stoi(config[11]);

  output_size = cv::Size(w, h);

  /* Initialize maps from CPU */
  xmap = cv::Mat(h, w, CV_32FC1);
  ymap = cv::Mat(h, w, CV_32FC1);

   //fill matrices
  cv::Mat cam(3, 3, cv::DataType<float>::type);
  cam.at<float>(0, 0) = fx;
  cam.at<float>(0, 1) = 0.0f;
  cam.at<float>(0, 2) = cx;

  cam.at<float>(1, 0) = 0.0f;
  cam.at<float>(1, 1) = fy;
  cam.at<float>(1, 2) = cy;

  cam.at<float>(2, 0) = 0.0f;
  cam.at<float>(2, 1) = 0.0f;
  cam.at<float>(2, 2) = 1.0f;

  cv::Mat dist(4, 1, cv::DataType<float>::type);
  dist.at<float>(0, 0) = k1;
  dist.at<float>(1, 0) = k2;
  dist.at<float>(2, 0) = p1;
  dist.at<float>(3, 0) = p2;

  if (fisheye == 1) {
    cv::fisheye::initUndistortRectifyMap(cam, dist, cv::Mat(), cam, cv::Size(w, h), CV_32FC1, xmap, ymap);
  }
  else {
    cv::initUndistortRectifyMap(cam, dist, cv::Mat(), cam, cv::Size(w, h), CV_32FC1, xmap, ymap);
  }

  printf("init camera distortion: %s, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f\n",
                                  config[0].c_str(), w, h, fx, fy, cx, cy, k1, k2, p1, p2);
}

int get_wrap_config(std::vector<std::string> config, cv::Mat &perspective, cv::Mat &affine, cv::Size &perspect_size) {
  perspective = cv::Mat(3, 3, cv::DataType<float>::type);
  affine = cv::Mat(2, 3, cv::DataType<float>::type);

  int do_wrap = std::stoi(config[12]);
  if (do_wrap != 0) {
    int perspective_width = std::stoi(config[13]);
    int perspective_height = std::stoi(config[14]);
    int affine_width = std::stoi(config[15]);
    int affine_height = std::stoi(config[16]);

    perspect_size = cv::Size(perspective_width, perspective_height);

    perspective.at<float>(0, 0) = std::stof(config[17]);
    perspective.at<float>(0, 1) = std::stof(config[18]);
    perspective.at<float>(0, 2) = std::stof(config[19]);

    perspective.at<float>(1, 0) = std::stof(config[20]);
    perspective.at<float>(1, 1) = std::stof(config[21]);
    perspective.at<float>(1, 2) = std::stof(config[22]);

    perspective.at<float>(2, 0) = std::stof(config[23]);
    perspective.at<float>(2, 1) = std::stof(config[24]);
    perspective.at<float>(2, 2) = std::stof(config[25]);

    affine.at<float>(0, 0) = std::stof(config[26]);
    affine.at<float>(0, 1) = std::stof(config[27]);
    affine.at<float>(0, 2) = std::stof(config[28]);

    affine.at<float>(1, 0) = std::stof(config[29]);
    affine.at<float>(1, 1) = std::stof(config[30]);
    affine.at<float>(1, 2) = std::stof(config[31]);

    printf("init camera wrap: %s, %d, %d, %d, %d, %d\n",
                              config[0].c_str(), do_wrap, perspective_width, perspective_height, affine_width, affine_height);
    printf("perspective: %s, [%f, %f, %f], [%f, %f, %f], [%f, %f, %f]\n",
                        config[0].c_str(), perspective.at<float>(0, 0), perspective.at<float>(0, 1), perspective.at<float>(0, 2),
                                            perspective.at<float>(1, 0), perspective.at<float>(1, 1), perspective.at<float>(1, 2),
                                            perspective.at<float>(2, 0), perspective.at<float>(2, 1), perspective.at<float>(2, 2));
    printf("affine: %s, [%f, %f, %f], [%f, %f, %f]\n",
                        config[0].c_str(), affine.at<float>(0, 0), affine.at<float>(0, 1), affine.at<float>(0, 2),
                                            affine.at<float>(1, 0), affine.at<float>(1, 1), affine.at<float>(1, 2));
  } else {
    printf("no panorama camera config\n");
  }
  return do_wrap;
}

static void undistortion( cv::Mat& image )
{
    pid_t token = gettid();
    Config_t cfg;
    std::string name = "";
    if (name_map.find(token) == name_map.end()) {
        std::vector<std::string> config = get_config();
        if (config.size() == 0) {
            std::cerr << "Error, No camera config found" << std::endl;
            return;
        }
        name = config[0];
        get_cv_remap(config, cfg.xmap, cfg.ymap, cfg.out_size);
        cfg.is_wrap = get_wrap_config(config, cfg.homography, cfg.affine, cfg.perspective_size);

        name_map[token] = name;
        config_map[name] = cfg;
    } else {
        name = name_map[token];
        cfg = config_map[name];
    }

    cv::Mat src, dst;
    cv::cvtColor(image, src, cv::COLOR_YUV2BGR_I420);
    if (cfg.is_wrap == 0) {
      cv::remap(src, dst, cfg.xmap, cfg.ymap, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0.f, 0.f, 0.f, 0.f));
      cv::cvtColor(dst, image, cv::COLOR_BGR2YUV_I420);
    } else {
    }
}