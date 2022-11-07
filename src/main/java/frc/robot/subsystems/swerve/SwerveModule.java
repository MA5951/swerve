package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    public abstract double getAbsoluteEncoderPosition();

    public abstract double getDrivePosition();

    public abstract double getTurningPosition();

    public abstract double getDriveVelocity();

    public abstract double getTurningVelocity();

    public abstract void resetEncoders();

    public abstract SwerveModuleState getState();
    
    public abstract void turningMotorSetPower(double power);

    public abstract void driveMotorSetPower(double power);

    public abstract void turningUsingPID(double setPoint);

    public abstract void driveUsingPID(double setPoint);

    public abstract void setDesiredState(SwerveModuleState desiredState);

    public abstract void stop();
}
