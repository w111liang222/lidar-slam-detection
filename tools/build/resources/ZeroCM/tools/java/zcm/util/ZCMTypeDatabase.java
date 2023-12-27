package zcm.util;

import java.util.*;

import javax.swing.*;
import javax.swing.event.*;
import javax.swing.table.*;
import javax.swing.tree.*;
import java.awt.*;
import java.awt.event.*;

import java.io.*;
import java.util.*;
import java.util.jar.*;
import java.util.zip.*;

import zcm.util.*;

import java.lang.reflect.*;

import zcm.zcm.*;

public class ZCMTypeDatabase
{
    HashMap<Long, Class> classes = new HashMap<Long, Class>();

    public ZCMTypeDatabase()
    {
        ClassDiscoverer.findClasses(new MyClassVisitor());
    }

    class MyClassVisitor implements ClassDiscoverer.ClassVisitor
    {
        public void classFound(String jar, Class cls)
        {
            try {
                Field[] fields = cls.getFields();

                for (Field f : fields) {
                    if (f.getName().equals("ZCM_FINGERPRINT")) {
                        // it's a static member, we don't need an instance
                        long fingerprint = f.getLong(null);
                        classes.put(fingerprint, cls);

                        break;
                    }
                }
            } catch (IllegalAccessException ex) {
                System.out.println("Bad ZCM Type? "+ex);
            }
        }
    }

    public Class getClassByFingerprint(long fingerprint)
    {
        return classes.get(fingerprint);
    }

    public int numFound() { return classes.size(); }

    public void print()
    {
        for (Long l : classes.keySet())
            System.out.println(Long.toHexString(l) + "   --->    " + classes.get(l));
    }
}
