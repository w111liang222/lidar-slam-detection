package zcm.logging;

import java.io.*;
import zcm.zcm.*;

/** A CLI implementation of a logger **/
public class Logger implements ZCMSubscriber
{
    Log log;
    Log.Event evt = new Log.Event();

    ZCM zcm;

    public Logger(String outfile) throws IOException
    {
        System.out.println("Logging to "+outfile);
        log = new Log(outfile, "rw");
        zcm = ZCM.getSingleton();
        zcm.subscribe(".*", this);
    }

    public void messageReceived(ZCM zcm, String channel, ZCMDataInputStream ins)
    {
        evt.utime = System.currentTimeMillis()*1000;
        evt.eventNumber = 0;
        evt.data = ins.getBuffer();
        evt.channel = channel;

        try {
            log.write(evt);
        } catch(IOException ex) {
            System.err.println("Failed to write event into log");
        }
    }

    public static void main(String[] args)
    {
        if (args.length != 1) {
            System.err.println("usage: lcm-logger <output-file>");
            System.exit(1);
        }

        try {
            new Logger(args[0]);
        } catch (IOException ex) {
            System.out.println("Err: failed to start logger");
            System.exit(1);
        }

        while (true) {
            try {
                Thread.sleep(100000);
            } catch(Exception ex) {}
        }
    }
}
