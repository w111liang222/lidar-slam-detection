package zcm.zcm;

import java.io.*;
import java.lang.Math;

public final class ZCMDataOutputStream implements DataOutput
{
    byte buf[];
    int pos_byte;
    int pos_bit;

    public ZCMDataOutputStream()
    {
        this(512);
    }

    public ZCMDataOutputStream(int sz)
    {
        this.buf = new byte[sz];
        this.reset();
    }

    public ZCMDataOutputStream(byte buf[])
    {
        this.buf = buf;
        this.reset();
    }

    public void resetBits()
    {
        if (pos_bit != 0) {
            pos_bit = 0;
            ++pos_byte;
        }
    }

    public void reset()
    {
        pos_byte = 0;
        pos_bit = 0;
    }

    void ensureSpace(int needed)
    {
        if (pos_byte+needed >= buf.length) {
            // compute new power-of-two capacity
            int newlen = buf.length;
            while (newlen < pos_byte+needed)
                newlen *= 2;

            byte buf2[] = new byte[newlen];
            System.arraycopy(buf, 0, buf2, 0, pos_byte);
            buf = buf2;
        }
    }

    public void write(byte b[])
    {
        ensureSpace(b.length);
        System.arraycopy(b, 0, buf, pos_byte, b.length);
        pos_byte += b.length;
    }

    public void write(byte b[], int off, int len)
    {
        ensureSpace(len);
        System.arraycopy(b, off, buf, pos_byte, len);
        pos_byte += len;
    }

    /** Writes one byte per char **/
    public void writeCharsAsBytes(char c[])
    {
        ensureSpace(c.length);
        for (int i = 0; i < c.length; i++)
            write(c[i]);
    }

    public void write(int b)
    {
        ensureSpace(1);
        buf[pos_byte++] = (byte) b;
    }

    public void writeBoolean(boolean v)
    {
        ensureSpace(1);
        buf[pos_byte++] = v ? (byte) 1 : (byte) 0;
    }

    public void writeByte(int v)
    {
        ensureSpace(1);
        buf[pos_byte++] = (byte) v;
    }

    public void writeBytes(String s)
    {
        ensureSpace(s.length());
        for (int i = 0; i < s.length(); i++) {
            buf[pos_byte++] = (byte) s.charAt(i);
        }
    }

    public void writeChar(int v)
    {
        writeShort(v);
    }

    public void writeChars(String s)
    {
        ensureSpace(2*s.length());
        for (int i = 0; i < s.length(); i++) {
            int v = s.charAt(i);
            buf[pos_byte++] = (byte) (v>>>8);
            buf[pos_byte++] = (byte) (v>>>0);
        }
    }

    /** Write a zero-terminated string consisting of 8 bit characters. **/
    public void writeStringZ(String s)
    {
        ensureSpace(s.length()+1);
        for (int i = 0; i < s.length(); i++) {
            buf[pos_byte++] = (byte) s.charAt(i);
        }
        buf[pos_byte++] = 0;
    }

    public void writeDouble(double v)
    {
        writeLong(Double.doubleToLongBits(v));
    }

    public void writeFloat(float v)
    {
        writeInt(Float.floatToIntBits(v));
    }

    public void writeInt(int v)
    {
        ensureSpace(4);
        buf[pos_byte++] = (byte) (v>>>24);
        buf[pos_byte++] = (byte) (v>>>16);
        buf[pos_byte++] = (byte) (v>>>8);
        buf[pos_byte++] = (byte) (v>>>0);
    }

    public void writeLong(long v)
    {
        ensureSpace(8);
        buf[pos_byte++] = (byte) (v>>>56);
        buf[pos_byte++] = (byte) (v>>>48);
        buf[pos_byte++] = (byte) (v>>>40);
        buf[pos_byte++] = (byte) (v>>>32);
        buf[pos_byte++] = (byte) (v>>>24);
        buf[pos_byte++] = (byte) (v>>>16);
        buf[pos_byte++] = (byte) (v>>>8);
        buf[pos_byte++] = (byte) (v>>>0);
    }

    public void writeShort(int v)
    {
        ensureSpace(2);
        buf[pos_byte++] = (byte) (v>>>8);
        buf[pos_byte++] = (byte) (v>>>0);
    }

    public void writeUTF(String s)
    {
        assert(false);
    }

    public void writeBits(long value, int numbits)
    {
        ensureSpace((int)Math.ceil((numbits + pos_bit) / 8));

        int bits_left = numbits;
        while (bits_left > 0) {
            if (pos_bit == 0) buf[pos_byte] = 0;
            int shift = pos_bit + bits_left - 8;
            if (shift < 0) {
                byte mask = (byte)((1 << bits_left) - 1);
                shift = -shift;
                buf[pos_byte] |= (value << shift) & (mask << shift);
                pos_bit += bits_left;
                break;
            }
            byte mask = (byte)(((short)1 << (bits_left - shift)) - 1);
            buf[pos_byte] |= (value >>> shift) & mask;
            bits_left = shift;
            pos_bit = 0;
            ++pos_byte;
        }
    }

    /** Makes a copy of the internal buffer. **/
    public byte[] toByteArray()
    {
        byte b[] = new byte[pos_byte];
        System.arraycopy(buf, 0, b, 0, pos_byte);
        return b;
    }

    /** Returns the internal buffer, which may be longer than the
     * buffer that has been written to so far.
     **/
    public byte[] getBuffer()
    {
        return buf;
    }

    /** Get the number of bytes that have been written to the buffer. **/
    public int size()
    {
        return pos_byte;
    }
}
