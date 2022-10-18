package frc.robot.subsystems.swerve;

public class SwerveConstants {
    private double distancePerPulse;
    private double anglePerPulse;
    private double velocityTimeUnitInSeconds;

    public SwerveConstants(double wheelRadius, int encoderResolution, double velocityTimeUnitInSeconds) {
        this.distancePerPulse = (2 * Math.PI * wheelRadius) / encoderResolution;
        this.anglePerPulse = (2 * Math.PI) / encoderResolution;
        this.velocityTimeUnitInSeconds = velocityTimeUnitInSeconds;
    }

    public double getDistancePerPulse() {
        return distancePerPulse;
    }

    public double getAnglePerPulse() {
        return anglePerPulse;
    }

    public double getVelocityTimeUnitInSeconds() {
        return velocityTimeUnitInSeconds;
    }
}
