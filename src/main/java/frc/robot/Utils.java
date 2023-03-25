package frc.robot;

public class Utils {
    public static double lerp(double factor, double min, double max) { return min + (max - min) * factor; }

    public static double unlerp(double value, double min, double max) { return (value - min) / (max - min); }
    
    public static double remap(double value, double fromMin, double fromMax, double toMin, double toMax) {
        return toMin + (toMax - toMin) * ((value - fromMin) / (fromMax - fromMin));
    }
}
