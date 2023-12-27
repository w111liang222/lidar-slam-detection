package zcm.zcm;

import java.io.*;

/** A class which listens for messages on a particular channel. **/
public interface ZCMSubscriber
{
    /**
     * Invoked by ZCM when a message is received.
     *
     * This method is invoked from the ZCM thread.
     *
     * @param zcm the ZCM instance that received the message.
     * @param channel the channel on which the message was received.
     * @param ins the message contents.
     */
    public void messageReceived(ZCM zcm, String channel, ZCMDataInputStream ins);
}
