package frc.robot.subsystems.swerve;

public class SwerveConstants {
    // swerve constants
    public final static double width = 0;
    public final static double length = 0;

    // Modules constants
    private final static double turningGearRatio = 1;
    private final static double driveGearRatio = 1;
    private final static int encoderResolution = 2048;
    private final static double wheelRadius = 0;
    
    public final static double velocityTimeUnitInSeconds = 0.1;

    public final static double distancePerPulse = (2 * wheelRadius * Math.PI)
        / (encoderResolution * driveGearRatio);
    public final static double anglePerPulse = 360 / 
        encoderResolution * turningGearRatio;
     
    // front left module
    public final static double frontLeftModuleOffsetEncoder = 0;
    public final static boolean frontLeftModuleIsDriveMotorReversed = false;
    public final static boolean frontLeftModuleIsTurningMotorReversed = false;
 
    // front right module
    public final static double frontRightModuleOffsetEncoder = 0;
    public final static boolean frontRightModuleIsDriveMotorReversed = false;
    public final static boolean frontRightModuleIsTurningMotorReversed = false;
 
    // rear left module
    public final static double rearLeftModuleOffsetEncoder = 0;
    public final static boolean rearLeftModuleIsDriveMotorReversed = false;
    public final static boolean rearLeftModuleIsTurningMotorReversed = false;
 
    // rear right module
    public final static double rearRightModuleOffsetEncoder = 0;
    public final static boolean rearRightModuleIsDriveMotorReversed = false;
    public final static boolean rearRightModuleIsTurningMotorReversed = false;

    // Modules turning config
    //PID
    public final static double turningPIDKP = 0;
    public final static double turningPIDKI = 0;
    public final static double turningPIDKD = 0;
    // Ramp
    public final static double openloopRamp = 0.25;
    public final static double closedloopRamp = 0;
    // Current Limit
    public static final int turningContinuousCurrentLimit = 25;
    public static final int turningPeakCurrentLimit = 40;
    public static final double turningPeakCurrentDuration = 0.1;
    public static final boolean turningEnableCurrentLimit = true;

    // Modules drive config
    //PID
    public final static double drivePIDKP = 0;
    public final static double drivePIDKI = 0;
    public final static double drivePIDKD = 0;
    public final static double driveKS = 0;
    public final static double driveKV = 0;
    // Current Limit
    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    // swerve controllers

    // swerve x P_CONTROLLER
    public final static double KP_X = 0;

    // swerve y P_CONTROLLER
    public final static double KP_Y = 0;

    // swerve theta PID_CONTROLLER
    public final static double theta_KP = 0;
    public final static double theta_KI = 0;
    public final static double theta_KD = 0;

    // swerve physics
    public final static double maxVelocity = 0;
    public final static double maxAcceleration = 0;
    public final static double maxAngularVelocity = 0; // radians
    public final static double maxAngularAcceleration = 0; // radians
}
