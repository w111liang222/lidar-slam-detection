package zcm.zcm;
import java.io.IOException;

class ZCMJNI
{
    static {
        System.loadLibrary("zcmjni");
    }

    private long nativePtr = 0;
    private native boolean initializeNative(String url);

    public ZCMJNI(String url) throws IOException
    {
        if (!initializeNative(url)) {
            String msg = (url != null) ?
                "Failed to create ZCM for '" + url + "'" :
                "Failed to create ZCM using the default url";
            throw new IOException(msg);
        }
    }

    public native void destroy();

    public native void start();
    public native void stop();

    public native int publish(String channel, byte[] data, int offset, int length);

    public native Object subscribe(String channel, ZCM zcm, Object usr);
    public native int unsubscribe(Object usr);
}
