package zcm.zcm;

import java.io.*;

/** Will not throw EOF. **/
public final class ZCMDataInputStream implements DataInput
{
    byte buf[];
    int pos_byte; // current index into buf.
    int pos_bit;  // current bit index
    int startpos; // index of first valid byte
    int endpos;   // index of byte after last valid byte

    public ZCMDataInputStream(byte buf[])
    {
        this(buf, 0, buf.length);
    }

    public ZCMDataInputStream(byte buf[], int offset, int len)
    {
        this.buf = buf;
        this.startpos = offset;
        this.endpos = offset + len + 1;
        this.reset();
    }

    void needInput(int need) throws EOFException
    {
        if (pos_byte + need >= endpos)
            throw new EOFException("ZCMDataInputStream needed "+need+" bytes, only "+available()+" available.");
    }

    public int available()
    {
        return endpos - pos_byte - 1;
    }

    public void close()
    {
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
        pos_byte = startpos;
        pos_bit = 0;
    }

    public boolean readBoolean() throws IOException
    {
        needInput(1);
        return (buf[pos_byte++]!=0);
    }

    public byte readByte() throws IOException
    {
        needInput(1);
        return buf[pos_byte++];
    }

    public int readUnsignedByte() throws IOException
    {
        needInput(1);
        return buf[pos_byte++]&0xff;
    }

    public char readChar() throws IOException
    {
        return (char) readShort();
    }

    public short readShort() throws IOException
    {
        needInput(2);
        return (short)
            (((buf[pos_byte++]&0xff) << 8) |
             ((buf[pos_byte++]&0xff) << 0));
    }

    public int readUnsignedShort() throws IOException
    {
        needInput(2);
        return
            ((buf[pos_byte++]&0xff) << 8) |
            ((buf[pos_byte++]&0xff) << 0);
    }

    public int readInt() throws IOException
    {
        needInput(4);
        return
            ((buf[pos_byte++]&0xff) << 24) |
            ((buf[pos_byte++]&0xff) << 16) |
            ((buf[pos_byte++]&0xff) << 8) |
            ((buf[pos_byte++]&0xff) << 0);
    }

    public long readLong() throws IOException
    {
        needInput(8);
        return
            ((buf[pos_byte++]&0xffL) << 56) |
            ((buf[pos_byte++]&0xffL) << 48) |
            ((buf[pos_byte++]&0xffL) << 40) |
            ((buf[pos_byte++]&0xffL) << 32) |
            ((buf[pos_byte++]&0xffL) << 24) |
            ((buf[pos_byte++]&0xffL) << 16) |
            ((buf[pos_byte++]&0xffL) << 8) |
            ((buf[pos_byte++]&0xffL) << 0);
    }

    public float readFloat() throws IOException
    {
        return Float.intBitsToFloat(readInt());
    }

    public void readFully(byte b[]) throws IOException
    {
        needInput(b.length);
        System.arraycopy(buf, pos_byte, b, 0, b.length);
        pos_byte += b.length;
    }

    public void readFully(byte b[], int off, int len) throws IOException
    {
        needInput(len);
        System.arraycopy(buf, pos_byte, b, off, len);
        pos_byte += len;
    }

    /** Writes chars as one byte per char, filling high byte with zero. **/
    public void readFullyBytesAsChars(char c[]) throws IOException
    {
        needInput(c.length);
        for (int i = 0; i < c.length; i++)
            c[i] = (char) (buf[pos_byte++]&0xff);
    }

    public double readDouble() throws IOException
    {
        return Double.longBitsToDouble(readLong());
    }

    public String readLine() throws IOException
    {
        StringBuffer sb = new StringBuffer();

        while (true) {
            needInput(1);
            byte v = buf[pos_byte++];
            if (v == 0)
                break;
            sb.append((char) v);
        }

        return sb.toString();
    }

    /** Read a string of 8-bit characters terminated by a zero. The zero is consumed. **/
    public String readStringZ() throws IOException
    {
        StringBuffer sb = new StringBuffer();
        while (true) {
            int v = buf[pos_byte++]&0xff;
            if (v == 0)
                break;
            sb.append((char) v);
        }

        return sb.toString();
    }

    public String readUTF() throws IOException
    {
        assert(false);
        return null;
    }

    public byte readByteBits(int numbits, boolean signExtend) throws IOException
    {
        byte ret = 0;
        int bits_left = numbits;
        while (bits_left > 0) {
            int available_bits = 8 - pos_bit;
            int bits_covered = available_bits < bits_left ? available_bits : bits_left;
            byte mask = (byte)(((1 << bits_covered) - 1) << (8 - bits_covered - pos_bit));
            byte payload = (byte)((buf[pos_byte] & mask) << pos_bit);
            int shift = 8 - bits_left;
            /* Sign extend the first shift and none after that */
            if (bits_left == numbits) {
                if (shift < 0) {
                    ret = (byte)(payload << -shift);
                } else {
                    ret = signExtend ?
                          (byte)(payload >> shift) :
                          (byte)((payload & 0xff) >>> shift);
                }
            } else {
                if (shift < 0) ret |= (byte)((payload & 0xff) << -shift);
                else           ret |= (byte)((payload & 0xff) >>> shift);
            }
            bits_left -= bits_covered;
            pos_bit += bits_covered;
            if (pos_bit == 8) {
                pos_bit = 0;
                ++pos_byte;
            }
        }
        return ret;
    }

    public short readShortBits(int numbits, boolean signExtend) throws IOException
    {
        short ret = 0;
        int bits_left = numbits;
        while (bits_left > 0) {
            int available_bits = 8 - pos_bit;
            int bits_covered = available_bits < bits_left ? available_bits : bits_left;
            byte mask = (byte)(((1 << bits_covered) - 1) << (8 - bits_covered - pos_bit));
            byte payload = (byte)((buf[pos_byte] & mask) << pos_bit);
            int shift = 8 - bits_left;
            /* Sign extend the first shift and none after that */
            if (bits_left == numbits) {
                if (shift < 0) {
                    ret = signExtend ?
                          (short)(payload << -shift) :
                          (short)((payload & 0xff) << -shift);
                } else {
                    ret = signExtend ?
                          (short)(payload >> shift) :
                          (short)((payload & 0xff) >>> shift);
                }
            } else {
                if (shift < 0) ret |= (short)((payload & 0xff) << -shift);
                else           ret |= (short)((payload & 0xff) >>> shift);
            }
            bits_left -= bits_covered;
            pos_bit += bits_covered;
            if (pos_bit == 8) {
                pos_bit = 0;
                ++pos_byte;
            }
        }
        return ret;
    }

    public int readIntBits(int numbits, boolean signExtend) throws IOException
    {
        int ret = 0;
        int bits_left = numbits;
        while (bits_left > 0) {
            int available_bits = 8 - pos_bit;
            int bits_covered = available_bits < bits_left ? available_bits : bits_left;
            byte mask = (byte)(((1 << bits_covered) - 1) << (8 - bits_covered - pos_bit));
            byte payload = (byte)((buf[pos_byte] & mask) << pos_bit);
            int shift = 8 - bits_left;
            /* Sign extend the first shift and none after that */
            if (bits_left == numbits) {
                if (shift < 0) {
                    ret = signExtend ?
                          (int)payload << -shift :
                          (payload & 0xff) << -shift;
                } else {
                    ret = signExtend ?
                          payload >> shift :
                          (payload & 0xff) >>> shift;
                }
            } else {
                if (shift < 0) ret |= (payload & 0xff) << -shift;
                else           ret |= (payload & 0xff) >>> shift;
            }
            bits_left -= bits_covered;
            pos_bit += bits_covered;
            if (pos_bit == 8) {
                pos_bit = 0;
                ++pos_byte;
            }
        }
        return ret;
    }

    public long readLongBits(int numbits, boolean signExtend) throws IOException
    {
        long ret = 0;
        int bits_left = numbits;
        while (bits_left > 0) {
            int available_bits = 8 - pos_bit;
            int bits_covered = available_bits < bits_left ? available_bits : bits_left;
            byte mask = (byte)(((1 << bits_covered) - 1) << (8 - bits_covered - pos_bit));
            byte payload = (byte)((buf[pos_byte] & mask) << pos_bit);
            int shift = 8 - bits_left;
            /* Sign extend the first shift and none after that */
            if (bits_left == numbits) {
                if (shift < 0) {
                    ret = signExtend ?
                          (long)payload << -shift :
                          (long)(payload & 0xff) << -shift;
                } else {
                    ret = signExtend ?
                          (long)(payload >> shift) :
                          (long)((payload & 0xff) >>> shift);
                }
            } else {
                if (shift < 0) ret |= (long)(payload & 0xff) << -shift;
                else           ret |= (long)(payload & 0xff) >>> shift;
            }
            bits_left -= bits_covered;
            pos_bit += bits_covered;
            if (pos_bit == 8) {
                pos_bit = 0;
                ++pos_byte;
            }
        }
        return ret;
    }

    public int skipBytes(int n)
    {
        pos_byte += n;
        return n;
    }

    /** Returns the internal buffer representation. **/
    public byte[] getBuffer()
    {
        return buf;
    }

    /** Returns the current position in the internal buffer representation. **/
    public int getBufferOffset()
    {
        return pos_byte;
    }
}
