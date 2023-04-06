package zcm.logging;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.io.PrintWriter;

public abstract class CsvWriterPlugin
{
    // return the array of fingerprints you can handle printing
    public abstract Long[] handleFingerprints();

    // Can call c.printZcmType or c.printArray for any submembers of the
    // zcmtype or types that this class can handle for default printing behavior
    // Returns number of lines written to csv
    public abstract int printCustom(String channel, Object o, long utime, PrintWriter output);

    public static int printDefault(String channel, Object o, long utime, PrintWriter output)
    {
        output.print(channel + ",");
        output.print(utime + ",");
        printZcmType(o, output);
        output.println("");
        return 1;
    }

    public static void printArray(Object o, PrintWriter output)
    {
        int length = Array.getLength(o);
        for (int i = 0; i < length; ++i) {
            Object item = Array.get(o, i);
            if (item.getClass().isArray()) printZcmType(item, output);
            else output.print(item + ",");
        }
    }

    public static void printZcmType(Object o, PrintWriter output)
    {
        Field fields[] = o.getClass().getFields();
        boolean isZcmType = false;
        for (Field field : fields) {
            String name = field.getName();
            // The first field should always be the fingerprint
            if(name == "ZCM_FINGERPRINT") {
                isZcmType = true;
                break;
            }
        }

        if (isZcmType == false) {
            if(o.getClass().isArray()) {
                printArray(o, output);
            } else {
                output.print(o + ",");
            }
            return;
        }

        for (Field field : fields) {
            String name = field.getName();

            // Dont want to print out the fingerprint
            if(name.startsWith("ZCM_FINGERPRINT"))
                continue;

            Object value = null;
            try {
                value = field.get(o);
            } catch(IllegalAccessException e) {
                System.err.println("Catastrophic error. Shoudln't be able to get here");
                System.exit(1);
            }
            output.print(name + ",");
            printZcmType(value, output);
        }
        output.flush();
    }
}
