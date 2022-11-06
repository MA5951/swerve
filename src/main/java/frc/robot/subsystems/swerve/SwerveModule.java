package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    public double getAbsoluteEncoderPosition();

    public double getDrivePosition();

    public double getTurningPosition();

    public double getDriveVelocity();

    public double getTurningVelocity();

    public void resetEncoders();

    public SwerveModuleState getState();
    
    public void turningMotorSetPower(double power);

    public void driveMotorSetPower(double power);

    public void turningUsingPID(double setPoint);

    public void driveUsingPID(double setPoint);

    public void setDesiredState(SwerveModuleState desiredState);

    public void stop();
}
