package frc.robot.util;

public class StackTrace555 
{
    public static StackTraceElement[] trace(int count) 
    {
        StackTraceElement[] trace;

        trace = Thread.currentThread().getStackTrace();
        trace = Array555.skip(trace, StackTraceElement[]::new, count + 1);

        return trace;
    }
    public static StackTraceElement[] trace() {return trace(0);}
}
