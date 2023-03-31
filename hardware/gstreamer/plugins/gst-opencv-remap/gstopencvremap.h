#ifndef __GST_OPENCV_REMAP_H__
#define __GST_OPENCV_REMAP_H__

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>

G_BEGIN_DECLS

#define GST_TYPE_OPENCV_REMAP (gst_opencv_remap_get_type())
G_DECLARE_FINAL_TYPE (GstOpencvRemap, gst_opencv_remap, GST, OPENCV_REMAP, GstBaseTransform)

struct _GstOpencvRemap {
  GstBaseTransform element;
  guint img_w;
  guint img_h;
};

G_END_DECLS

#endif /* __GST_OPENCV_REMAP_H__ */