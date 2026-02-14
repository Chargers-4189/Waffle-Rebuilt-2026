package frc.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.FlyWheelConstants;

public class NetworkTables {
    static NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();

    public static final class FlyTable {
        static NetworkTable flyTable = networkInstance.getTable("flyTable");
        public static DoubleEntry kS = flyTable.getDoubleTopic("kS").getEntry(FlyWheelConstants.kS);; // Add 0.25 V output to overcome static friction
        public static DoubleEntry kV = flyTable.getDoubleTopic("kV").getEntry(FlyWheelConstants.kV);; // A velocity target of 1 rps results in 0.12 V output
        public static DoubleEntry kA = flyTable.getDoubleTopic("kA").getEntry(FlyWheelConstants.kA);; // An acceleration of 1 rps/s requires 0.01 V output
        public static DoubleEntry kP = flyTable.getDoubleTopic("kP").getEntry(FlyWheelConstants.kP);
        public static DoubleEntry kI = flyTable.getDoubleTopic("kI").getEntry(FlyWheelConstants.kI);
        public static DoubleEntry kD = flyTable.getDoubleTopic("kD").getEntry(FlyWheelConstants.kD);

        // set Motion Magic settings
        public static DoubleEntry MotionMagicAcceleration = flyTable.getDoubleTopic("MotionMagicAcceleration").getEntry(FlyWheelConstants.MotionMagicAcceleration);; // Target acceleration of 160 rps/s (0.5 seconds)
        public static DoubleEntry MotionMagicJerk = flyTable.getDoubleTopic("MotionMagicJerk").getEntry(FlyWheelConstants.MotionMagicJerk);; // Target jerk of 1600 rps/s/s (0.1 seconds)
    }
}