package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    public abstract double getAbsoluteEncoderPosition();

    public abstract double getDrivePosition();

    public abstract double getTurningPosition();

    public abstract double getDriveVelocity();

    public abstract double getTurningVelocity();

    public abstract void resetEncoders();

    public abstract void turningMotorSetPower(double power);

    public abstract void driveMotorSetPower(double power);

    public abstract void turningUsingPID(double setPoint);

    public abstract void driveUsingPID(double setPoint);

    public abstract void setNeutralMode(NeutralMode mode);

    public abstract void setInvertedTurning(Boolean mode);

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(), new Rotation2d(Math.toRadians(getTurningPosition()))
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(), new Rotation2d(Math.toRadians(getTurningPosition()))
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModule.optimize(desiredState,
                getTurningPosition(), getDriveVelocity());
        if (optimizedState.speedMetersPerSecond != 0) {
            turningUsingPID(optimizedState.angle.getDegrees());
        } else {
            turningMotorSetPower(0);
        }
        driveUsingPID(optimizedState.speedMetersPerSecond);
    }

    public void stop() {
        driveMotorSetPower(0);
        turningMotorSetPower(0);
    }

    private static SwerveModuleState optimize(SwerveModuleState desiredState, 
    double currentAngle, double currV) {
        double angleDiff = (desiredState.angle.getDegrees() - currentAngle) % 360;
        double targetAngle = currentAngle + angleDiff;
        double targetSpeed = desiredState.speedMetersPerSecond;

        if (angleDiff <= -270) {
            targetAngle += 360;
        } else if (-90 > angleDiff && angleDiff > -270) {
            targetAngle += 180;
            targetSpeed = -targetSpeed;
        } else if (90 < angleDiff && angleDiff < 270) {
            targetAngle -= 180;
            targetSpeed = -targetSpeed;
        } else if (angleDiff >= 270) {
            targetAngle -= 360;
        }
        
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }
}
