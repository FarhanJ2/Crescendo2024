package frc.robot.regression;

import frc.team254.lib.util.InterpolatingDouble;
import frc.team254.lib.util.InterpolatingTreeMap;
import frc.team254.lib.util.PolynomialRegression;

public class ShooterInterpolator {

    // Pivot
    public static double kDefaultPivotAngle = Math.toRadians(0);
    public static boolean kUsePivotAutoAimPolynomial = false;

    public static boolean kUseSmartdashboard = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPivotAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kPivotAutoAimPolynomial;

    public static double[][] kPivotRegression;

    static {
        kPivotRegression = RegressionData.kPivotAngle;
        //iterate through the array and place each point into the interpolating tree
        for (double[] pair : kPivotRegression) {
            kPivotAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        
        kPivotAutoAimPolynomial = new PolynomialRegression(kPivotRegression, 1);
    }
    
    // Shooter
    public static double kDefaultShootingRPM = 0;
    public static boolean kUseFlywheelAutoAimPolynomial = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kFlywheelAutoAimPolynomial;

    public static double[][] kFlywheelRegression;

    static {
        kFlywheelRegression = RegressionData.kFlywheelRPM;

        for (double[] pair : kFlywheelRegression) {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelRegression, 2);
    }

}
