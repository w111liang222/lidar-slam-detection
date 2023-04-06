package zcm.logging;

import java.util.ArrayList;

import zcm.logging.*;

public abstract class CsvReaderPlugin
{
    public abstract ArrayList<Log.Event> readZcmType(String line);
}
