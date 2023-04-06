package zcm.logging;

import java.util.ArrayList;

import zcm.logging.*;

public abstract class TranscoderPlugin
{
    // return the array of fingerprints you can handle printing
    public abstract Long[] handleFingerprints();

    // return an ArrayList of Log.Events to be written to the new log instead
    // of the Log.Event input argument
    public abstract ArrayList<Log.Event> transcodeMessage(String channel,
                                                          Object o,
                                                          long utime,
                                                          Log.Event ev);
}
