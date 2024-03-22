package frc.robot.regression;

// clapped
public class RegressionData {
    public static double[][] kPivotAngle = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> hood angle (in degrees)
        // TODO get actual data
        { 0.25, 0.4 },
        { 0.5, 0.35 },
        { 0.75, 0.32 },
        { 1.0, 0.3 },
        { 1.25, 0.27 },
        { 1.5, 0.24 },
        { 1.75, 0.22 },
        { 2.0, 0.2 },
        { 2.25, 0.19 },
        { 2.5, 0.17 },
        { 2.75, 0.16 },
        { 3.0, 0.15 },
        { 3.25, 0.14 },
        { 3.5, 0.12 },
    };

    public static double[][] kFlywheelRPM = {
        /* TEMPLATE REGRESSION */
        // @x --> distance from target (in meters)
        // @y --> shooter velocity (in rpm)
        // TODO get actual data
        // { 0.25, 2000 },
        // { 0.5, 2200 },
        // { 0.75, 2350 },
        // { 1.0, 2400 },
        // { 1.25, 2450 },
        // { 1.5, 2500 },
        // { 1.75, 2600 },
        // { 2.0, 2700 },
        // { 2.25, 2800 },
        // { 2.5, 2900 },
        // { 2.75, 3000 },
        // { 3.0, 3100 },
        // { 3.25, 3200 },
        // { 3.5, 3400 },
        { 0.25, 2000 - 1000 },
        { 0.5, 2200 - 1000 },
        { 0.75, 2350 - 1000 },
        { 1.0, 2400 - 1000 },
        { 1.25, 2450 - 1000 },
        { 1.5, 2500 - 1000 },
        { 1.75, 2600 - 1000 },
        { 2.0, 2700 - 1000 },
        { 2.25, 2800 - 1000 },
        { 2.5, 2900 - 1000 },
        { 2.75, 3000 - 1000 },
        { 3.0, 3100 - 1000 },
        { 3.25, 3200 - 1000 },
        { 3.5, 3400 - 1000 },
    };
}