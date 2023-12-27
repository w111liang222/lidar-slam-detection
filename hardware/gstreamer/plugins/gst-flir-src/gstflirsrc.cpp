#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>
#include <gst/video/video.h>

#include "gstflirsrc.h"

GST_DEBUG_CATEGORY_STATIC (gst_flirsrc_debug);
#define GST_CAT_DEFAULT gst_flirsrc_debug

/* prototypes */
static void gst_flirsrc_set_property (GObject * object,
                                     guint property_id, const GValue * value, GParamSpec * pspec);
static void gst_flirsrc_get_property (GObject * object,
                                     guint property_id, GValue * value, GParamSpec * pspec);
static void gst_flirsrc_dispose (GObject * object);
static void gst_flirsrc_finalize (GObject * object);

static gboolean gst_flirsrc_start (GstBaseSrc * src);
static gboolean gst_flirsrc_stop (GstBaseSrc * src);
static GstCaps* gst_flirsrc_get_caps (GstBaseSrc * src, GstCaps * filter);
static gboolean gst_flirsrc_set_caps (GstBaseSrc * src, GstCaps * caps);
static gboolean gst_flirsrc_unlock (GstBaseSrc * src);
static gboolean gst_flirsrc_unlock_stop (GstBaseSrc * src);

static GstFlowReturn gst_flirsrc_fill (GstPushSrc * src, GstBuffer * buf);

enum
{
    PROP_0,
    PROP_CAM_DEV,
    PROP_CAM_RES,
    PROP_CAM_FPS,
    N_PROPERTIES
};

typedef enum {
    GST_FLIRSRC_30FPS = 30,
} GstZedSrcFPS;

//////////////// DEFAULT PARAMETERS //////////////////////////////////////////////////////////////////////////

// INITIALIZATION
#define DEFAULT_PROP_CAM_DEV                    static_cast<gint>(0)
#define DEFAULT_PROP_CAM_RES                    static_cast<gint>(0)
#define DEFAULT_PROP_CAM_FPS                    GST_FLIRSRC_30FPS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define GST_TYPE_FLIR_RESOL (gst_flirsrc_resol_get_type ())
static GType gst_flirsrc_resol_get_type (void)
{
    static GType flirsrc_resol_type = 0;

    if (!flirsrc_resol_type) {
        static GEnumValue pattern_types[] = {
            { static_cast<gint>(0),
              "1280x768",
              "720P" },
            { 0, NULL, NULL },
        };

        flirsrc_resol_type = g_enum_register_static( "GstFlirsrcResolution",
                                                    pattern_types);
    }

    return flirsrc_resol_type;
}

#define GST_TYPE_FLIR_FPS (gst_flirsrc_fps_get_type ())
static GType gst_flirsrc_fps_get_type (void)
{
    static GType flirsrc_fps_type = 0;

    if (!flirsrc_fps_type) {
        static GEnumValue pattern_types[] = {
            { GST_FLIRSRC_30FPS,
              "Default FPS",
              "30 FPS" },
            { 0, NULL, NULL },
        };

        flirsrc_fps_type = g_enum_register_static( "GstFlirSrcFPS",
                                                  pattern_types);
    }

    return flirsrc_fps_type;
}

/* pad templates */
static GstStaticPadTemplate gst_flirsrc_src_template =
        GST_STATIC_PAD_TEMPLATE ("src",
                                 GST_PAD_SRC,
                                 GST_PAD_ALWAYS,
                                 GST_STATIC_CAPS( ("video/x-raw, "
                                                   "format = (string)I420, "
                                                   "width = (int)1280, "
                                                   "height =  (int)768, "
                                                   "framerate = (fraction) { 10, 20, 30 }") ) );

/* class initialization */
G_DEFINE_TYPE( GstFlirSrc, gst_flirsrc, GST_TYPE_PUSH_SRC );

static void gst_flirsrc_class_init (GstFlirSrcClass * klass)
{
    GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
    GstElementClass *gstelement_class = GST_ELEMENT_CLASS (klass);
    GstBaseSrcClass *gstbasesrc_class = GST_BASE_SRC_CLASS (klass);
    GstPushSrcClass *gstpushsrc_class = GST_PUSH_SRC_CLASS (klass);

    gobject_class->set_property = gst_flirsrc_set_property;
    gobject_class->get_property = gst_flirsrc_get_property;
    gobject_class->dispose = gst_flirsrc_dispose;
    gobject_class->finalize = gst_flirsrc_finalize;

    gst_element_class_add_pad_template (gstelement_class,
                                        gst_static_pad_template_get (&gst_flirsrc_src_template));

    gst_element_class_set_static_metadata (gstelement_class,
                                           "FLIR Camera Source",
                                           "Source/Video",
                                           "FLIR Camera source",
                                           "LiangWang <15lwang@alumni.tongji.edu.cn>");

    gstbasesrc_class->start = GST_DEBUG_FUNCPTR (gst_flirsrc_start);
    gstbasesrc_class->stop = GST_DEBUG_FUNCPTR (gst_flirsrc_stop);
    gstbasesrc_class->get_caps = GST_DEBUG_FUNCPTR (gst_flirsrc_get_caps);
    gstbasesrc_class->set_caps = GST_DEBUG_FUNCPTR (gst_flirsrc_set_caps);
    gstbasesrc_class->unlock = GST_DEBUG_FUNCPTR (gst_flirsrc_unlock);
    gstbasesrc_class->unlock_stop = GST_DEBUG_FUNCPTR (gst_flirsrc_unlock_stop);

    gstpushsrc_class->fill = GST_DEBUG_FUNCPTR (gst_flirsrc_fill);

    /* Install GObject properties */
    g_object_class_install_property( gobject_class, PROP_CAM_DEV,
                                     g_param_spec_int("device", "Camera Device Number",
                                                      "Select camera from cameraID",0,255,
                                                      DEFAULT_PROP_CAM_DEV,
                                                      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property( gobject_class, PROP_CAM_RES,
                                     g_param_spec_enum("camera-resolution", "Camera Resolution",
                                                       "Camera Resolution", GST_TYPE_FLIR_RESOL, DEFAULT_PROP_CAM_RES,
                                                       (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property( gobject_class, PROP_CAM_FPS,
                                     g_param_spec_enum("camera-fps", "Camera frame rate",
                                                       "Camera frame rate", GST_TYPE_FLIR_FPS, DEFAULT_PROP_CAM_FPS,
                                                       (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
}

static void gst_flirsrc_reset (GstFlirSrc * src)
{
    if(src->cap != nullptr) {
        src->cap.reset();
    }

    src->out_framesize = 0;
    src->is_started = FALSE;

    src->last_frame_count = 0;
    src->total_dropped_frames = 0;

    if (src->caps) {
        gst_caps_unref (src->caps);
        src->caps = NULL;
    }
}

static void gst_flirsrc_init (GstFlirSrc * src)
{
    /* set source as live (no preroll) */
    gst_base_src_set_live (GST_BASE_SRC (src), TRUE);

    /* override default of BYTES to operate in time mode */
    gst_base_src_set_format (GST_BASE_SRC (src), GST_FORMAT_TIME);

    // ----> Parameters initialization
    src->camera_device = DEFAULT_PROP_CAM_DEV;
    src->camera_resolution = DEFAULT_PROP_CAM_RES;
    src->camera_fps = DEFAULT_PROP_CAM_FPS;
    // <---- Parameters initialization

    src->stop_requested = FALSE;
    src->caps = NULL;

    gst_flirsrc_reset (src);
}

void gst_flirsrc_set_property (GObject * object, guint property_id,
                              const GValue * value, GParamSpec * pspec)
{
    GstFlirSrc *src;
    const gchar* str;

    src = GST_FILR_SRC(object);

    switch (property_id) {
    case PROP_CAM_DEV:
        src->camera_device = g_value_get_int(value);
        break;
    case PROP_CAM_RES:
        src->camera_resolution = g_value_get_enum(value);
        break;
    case PROP_CAM_FPS:
        src->camera_fps = g_value_get_enum(value);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
        break;
    }
}

void
gst_flirsrc_get_property (GObject * object, guint property_id,
                         GValue * value, GParamSpec * pspec)
{
    GstFlirSrc *src;

    g_return_if_fail (GST_IS_FLIR_SRC (object));
    src = GST_FILR_SRC (object);

    switch (property_id) {
    case PROP_CAM_DEV:
        g_value_set_int( value, src->camera_device );
        break;
    case PROP_CAM_RES:
        g_value_set_enum( value, src->camera_resolution );
        break;
    case PROP_CAM_FPS:
        g_value_set_enum( value, src->camera_fps );
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
        break;
    }
}

void
gst_flirsrc_dispose (GObject * object)
{
    GstFlirSrc *src;

    g_return_if_fail (GST_IS_FLIR_SRC (object));
    src = GST_FILR_SRC (object);

    /* clean up as possible.  may be called multiple times */

    G_OBJECT_CLASS (gst_flirsrc_parent_class)->dispose (object);
}

void
gst_flirsrc_finalize (GObject * object)
{
    GstFlirSrc *src;

    g_return_if_fail (GST_IS_FLIR_SRC (object));
    src = GST_FILR_SRC (object);

    /* clean up object here */
    if (src->caps) {
        gst_caps_unref (src->caps);
        src->caps = NULL;
    }

    G_OBJECT_CLASS (gst_flirsrc_parent_class)->finalize (object);
}

static gboolean gst_flirsrc_calculate_caps(GstFlirSrc* src)
{
    guint32 width, height;
    gint fps;
    GstVideoInfo vinfo;
    GstVideoFormat format = GST_VIDEO_FORMAT_I420;

    width = src->cap->get(cv::CAP_PROP_FRAME_WIDTH);
    height = src->cap->get(cv::CAP_PROP_FRAME_HEIGHT);
    fps = src->cap->get(cv::CAP_PROP_FPS);

    gst_video_info_init( &vinfo );
    gst_video_info_set_format( &vinfo, format, width, height );
    if (src->caps) {
        gst_caps_unref (src->caps);
    }
    src->out_framesize = (guint) GST_VIDEO_INFO_SIZE (&vinfo);
    vinfo.fps_n = fps;
    vinfo.fps_d = 1;
    src->caps = gst_video_info_to_caps (&vinfo);

    gst_base_src_set_blocksize( GST_BASE_SRC(src), src->out_framesize );
    gst_base_src_set_caps ( GST_BASE_SRC(src), src->caps );
    GST_DEBUG_OBJECT( src, "Created caps %" GST_PTR_FORMAT, src->caps );

    return TRUE;
}

static gboolean gst_flirsrc_start( GstBaseSrc * bsrc )
{
    GstFlirSrc *src = GST_FILR_SRC (bsrc);

    GST_DEBUG_OBJECT( src, "start" );

    GST_INFO("CAMERA INITIALIZATION PARAMETERS");

    src->cap = toy::createSpinnakerCapture(src->camera_device);
    if (src->cap == nullptr) {
        return FALSE;
    }

    if (!gst_flirsrc_calculate_caps(src) ) {
        return FALSE;
    }

    return TRUE;
}

static gboolean gst_flirsrc_stop (GstBaseSrc * bsrc)
{
    GstFlirSrc *src = GST_FILR_SRC (bsrc);

    GST_DEBUG_OBJECT (src, "stop");

    gst_flirsrc_reset( src );

    return TRUE;
}

static GstCaps* gst_flirsrc_get_caps( GstBaseSrc * bsrc, GstCaps * filter )
{
    GstFlirSrc *src = GST_FILR_SRC (bsrc);
    GstCaps *caps;

    if (src->caps)
    {
        caps = gst_caps_copy (src->caps);
    }
    else
    {
        caps = gst_pad_get_pad_template_caps (GST_BASE_SRC_PAD (src));
    }

    GST_DEBUG_OBJECT (src, "The caps before filtering are %" GST_PTR_FORMAT,
                      caps);

    if (filter && caps)
    {
        GstCaps *tmp = gst_caps_intersect( caps, filter );
        gst_caps_unref (caps);
        caps = tmp;
    }

    GST_DEBUG_OBJECT (src, "The caps after filtering are %" GST_PTR_FORMAT, caps);

    return caps;
}

static gboolean gst_flirsrc_set_caps( GstBaseSrc * bsrc, GstCaps * caps )
{
    GstFlirSrc *src = GST_FILR_SRC (bsrc);
    GstVideoInfo vinfo;

    gst_caps_get_structure( caps, 0 );

    GST_DEBUG_OBJECT (src, "The caps being set are %" GST_PTR_FORMAT, caps);

    gst_video_info_from_caps (&vinfo, caps);

    if (GST_VIDEO_INFO_FORMAT (&vinfo) == GST_VIDEO_FORMAT_UNKNOWN)
    {
        goto unsupported_caps;
    }

    return TRUE;

unsupported_caps:
    GST_ERROR_OBJECT (src, "Unsupported caps: %" GST_PTR_FORMAT, caps);
    return FALSE;
}

static gboolean gst_flirsrc_unlock( GstBaseSrc * bsrc )
{
    GstFlirSrc *src = GST_FILR_SRC (bsrc);

    GST_LOG_OBJECT (src, "unlock");

    src->stop_requested = TRUE;

    return TRUE;
}

static gboolean gst_flirsrc_unlock_stop( GstBaseSrc * bsrc )
{
    GstFlirSrc *src = GST_FILR_SRC (bsrc);

    GST_LOG_OBJECT (src, "unlock_stop");

    src->stop_requested = FALSE;

    return TRUE;
}

static GstFlowReturn gst_flirsrc_fill( GstPushSrc * psrc, GstBuffer * buf )
{
    GstFlirSrc *src = GST_FILR_SRC (psrc);
    GstMapInfo minfo;
    GstClock *clock;
    GstClockTime clock_time;

    static int temp_ugly_buf_index = 0;

    GST_LOG_OBJECT (src, "fill");
    if (!src->is_started) {
        src->acq_start_time =
                gst_clock_get_time(gst_element_get_clock (GST_ELEMENT (src)));

        src->is_started = TRUE;
    }

    cv::Mat im;
    if (!src->cap->read(im)) {
        return GST_FLOW_ERROR;
    }

    // ----> Clock update
    clock = gst_element_get_clock (GST_ELEMENT (src));
    clock_time = gst_clock_get_time (clock);
    gst_object_unref (clock);
    // <---- Clock update

    // Memory mapping
    if( FALSE==gst_buffer_map( buf, &minfo, GST_MAP_WRITE ) )
    {
        GST_ELEMENT_ERROR (src, RESOURCE, FAILED,
                           ("Failed to map buffer for writing" ), (NULL));
        return GST_FLOW_ERROR;
    }

    memcpy(minfo.data, im.data, minfo.size);

    // ----> Timestamp meta-data
    GST_BUFFER_TIMESTAMP(buf) = GST_CLOCK_DIFF (gst_element_get_base_time (GST_ELEMENT (src)),
                                                clock_time);
    GST_BUFFER_DTS(buf) = GST_BUFFER_TIMESTAMP(buf);
    GST_BUFFER_OFFSET(buf) = temp_ugly_buf_index++;
    // <---- Timestamp meta-data

    gst_buffer_unmap( buf, &minfo );

    if (src->stop_requested) {
        return GST_FLOW_FLUSHING;
    }

    return GST_FLOW_OK;
}


static gboolean plugin_init (GstPlugin * plugin)
{
    GST_DEBUG_CATEGORY_INIT( gst_flirsrc_debug, "flirsrc", 0,
                             "debug category for flirsrc element");
    gst_element_register( plugin, "flirsrc", GST_RANK_NONE,
                          gst_flirsrc_get_type());

    return TRUE;
}

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
                   GST_VERSION_MINOR,
                   flirsrc,
                   "Flir camera source",
                   plugin_init,
                   GST_PACKAGE_VERSION,
                   GST_PACKAGE_LICENSE,
                   GST_PACKAGE_NAME,
                   GST_PACKAGE_ORIGIN)
