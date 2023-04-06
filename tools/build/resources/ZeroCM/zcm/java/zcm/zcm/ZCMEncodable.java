package zcm.zcm;

import java.io.*;

/** A message which can be easily sent using ZCM.
 **/
public interface ZCMEncodable
{
    /** ZCMEncodables will always have an empty constructor and a
     * constructor that takes a DataInput. **/

    /**
     * Invoked by ZCM.
     * @param outs Any data to be sent should be written to this output stream.
     */
    public void encode(ZCMDataOutputStream outs) throws IOException;


    /** Encode the data without the magic header. Most users will
     * never use this function.
     **/
    public void _encodeRecursive(ZCMDataOutputStream outs) throws IOException;

    /** Decode the data without the magic header. Most users will
     * never use this function.
     **/
    public void _decodeRecursive(ZCMDataInputStream ins) throws IOException;

}
