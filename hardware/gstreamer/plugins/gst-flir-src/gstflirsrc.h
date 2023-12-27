#ifndef _GST_FLIR_SRC_H_
#define _GST_FLIR_SRC_H_

#include <video_capture.h>
#include <gst/base/gstpushsrc.h>

G_BEGIN_DECLS

#define GST_TYPE_FLIR_SRC   (gst_flirsrc_get_type())
#define GST_FILR_SRC(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_FLIR_SRC,GstFlirSrc))
#define GST_FLIR_SRC_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_FLIR_SRC,GstFlirSrcClass))
#define GST_IS_FLIR_SRC(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_FLIR_SRC))
#define GST_IS_FLIR_SRC_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_FLIR_SRC))

typedef struct _GstFlirSrc GstFlirSrc;
typedef struct _GstFlirSrcClass GstFlirSrcClass;

struct _GstFlirSrc
{
    GstPushSrc base_flirsrc;

    // FLIR driver object
    toy::VideoCapture::Ptr cap;

    gboolean is_started; // grab started flag

    // ----> Properties
    gint camera_device;
    gint camera_resolution;     // Camera resolution [enum]
    gint camera_fps;            // Camera FPS
    // <---- Properties

    GstClockTime acq_start_time;
    guint32 last_frame_count;
    guint32 total_dropped_frames;

    GstCaps *caps;
    guint out_framesize;

    gboolean stop_requested;
};

struct _GstFlirSrcClass
{
    GstPushSrcClass base_flirsrc_class;
};

G_GNUC_INTERNAL GType gst_flirsrc_get_type (void);

G_END_DECLS

#endif
