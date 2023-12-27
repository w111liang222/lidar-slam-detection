package zcm.zcm;

import java.net.*;
import java.io.*;
import java.util.*;
import java.nio.*;

/** Zero Communications and Marshalling Java implementation **/
public class ZCM
{
    public class Subscription
    {
        Object nativeSub;
        ZCMSubscriber javaSub;
    }

    boolean closed = false;

    static ZCM singleton;

    ZCMDataOutputStream encodeBuffer = new ZCMDataOutputStream(new byte[1024]);
    ZCMJNI zcmjni;

    /** Create a new ZCM object, connecting to one or more URLs. If
     * no URL is specified, ZCM_DEFAULT_URL is used.
     **/
    public ZCM() throws IOException { this(null); }
    public ZCM(String url) throws IOException
    {
        zcmjni = new ZCMJNI(url);
    }

    public void start() { zcmjni.start(); }
    public void stop()  { zcmjni.stop(); }

    /** Retrieve a default instance of ZCM using either the environment
     * variable ZCM_DEFAULT_URL or the default. If an exception
     * occurs, System.exit(-1) is called.
     **/
    public static ZCM getSingleton()
    {
        if (singleton == null) {
            try {
                // TODO: add back in capability to use the ZCM_DEFAULT_URL env variable
                //       as the default for getSingleton
                singleton = new ZCM();
            } catch (Exception ex) {
                System.err.println("ZCM singleton fail: "+ex);
                System.exit(-1);
                return null;
            }
        }

        return singleton;
    }

    /** Publish a string on a channel. This method does not use the
     * ZCM type definitions and thus is not type safe. This method is
     * primarily provided for testing purposes and may be removed in
     * the future.
     **/
    public void publish(String channel, String s) throws IOException
    {
        if (this.closed) throw new IllegalStateException();
        s = s + "\0";
        byte[] b = s.getBytes();
        publish(channel, b, 0, b.length);
    }

    /** Publish an ZCM-defined type on a channel. If more than one URL was
     * specified, the message will be sent on each.
     **/
    public synchronized void publish(String channel, ZCMEncodable e)
    {
        if (this.closed) throw new IllegalStateException();

        try {
            encodeBuffer.reset();

            e.encode(encodeBuffer);

            publish(channel, encodeBuffer.getBuffer(), 0, encodeBuffer.size());
        } catch (IOException ex) {
            System.err.println("ZCM publish fail: "+ex);
        }
    }

    /** Publish raw data on a channel, bypassing the ZCM type
     * specification. If more than one URL was specified when the ZCM
     * object was created, the message will be sent on each.
     **/
    public void publish(String channel, byte[] data, int offset, int length)
        throws IOException
    {
        if (this.closed) throw new IllegalStateException();
        zcmjni.publish(channel, data, offset, length);
    }

    public Subscription subscribe(String channel, ZCMSubscriber sub)
    {
        if (this.closed) throw new IllegalStateException();

        Subscription subs = new Subscription();
        subs.javaSub = sub;

        subs.nativeSub = zcmjni.subscribe(channel, this, subs);

        return subs;
    }

    public int unsubscribe(Subscription subs) {
        if (this.closed) throw new IllegalStateException();
        return zcmjni.unsubscribe(subs.nativeSub);
    }

    /** Not for use by end users. Provider back ends call this method
     * when they receive a message. The subscribers that match the
     * channel name are synchronously notified.
     **/
    public void receiveMessage(String channel,
                               byte data[], int offset, int length,
                               Subscription subs)
    {
        if (this.closed) throw new IllegalStateException();
        subs.javaSub.messageReceived(this,
                                     channel,
                                     new ZCMDataInputStream(data, offset, length));
    }

    /** Call this function to release all resources used by the ZCM instance.  After calling this
     * function, the ZCM instance should consume no resources, and cannot be used to
     * receive or transmit messages.
     */
    public void close()
    {
        if (this.closed) throw new IllegalStateException();
        zcmjni.destroy();
        this.closed = true;
    }

    public void finalize() { if (!this.closed) close(); }

    ////////////////////////////////////////////////////////////////

    /** Minimalist test code.
     **/
    public static void main(String args[])
    {
        ZCM zcm;

        try {
            zcm = new ZCM();
        } catch (IOException ex) {
            System.err.println("ex: "+ex);
            return;
        }

        SimpleSubscriber subscriber1 = new SimpleSubscriber();
        SimpleSubscriber subscriber2 = new SimpleSubscriber();

        ZCM.Subscription subs1 = zcm.subscribe(".*", subscriber1);
        ZCM.Subscription subs2 = zcm.subscribe(".*", subscriber2);

        int numMsgsSent = 0;

        zcm.start();
        while (true) {
            if (subscriber1.getNumMsgsReceived() >= 10)
                break;
            try {
                Thread.sleep(250);
                zcm.publish("TEST", "foobar");
                numMsgsSent++;
            } catch (Exception ex) {
                System.err.println("ex: "+ex);
            }
        }
        zcm.stop();

        zcm.unsubscribe(subs1);
        zcm.unsubscribe(subs2);

        zcm.close();
    }

    static class SimpleSubscriber implements ZCMSubscriber
    {
        public int numMsgsReceived = 0;

        public synchronized int getNumMsgsReceived() { return numMsgsReceived; }

        public void messageReceived(ZCM zcm, String channel, ZCMDataInputStream dins)
        {
            synchronized (this) {
                numMsgsReceived++;
            }
            System.err.println("RECV: "+channel);
        }
    }
}
