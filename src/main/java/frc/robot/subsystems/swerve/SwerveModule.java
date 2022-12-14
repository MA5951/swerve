package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    public abstract double getAbsoluteEncoderPosition();

    public abstract double getDrivePosition();

    public abstract double getTurningPosition();

    public abstract double getDriveVelocity();

    public abstract double getTurningVelocity();

    public abstract void resetEncoders();

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
                new Rotation2d(Math.toRadians(getTurningPosition())));
    }
    
    public abstract void turningMotorSetPower(double power);

    public abstract void driveMotorSetPower(double power);

    public abstract void turningUsingPID(double setPoint);

    public abstract void driveUsingPID(double setPoint);

    public abstract void setNeutralMode(NeutralMode mode);

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

    public static SwerveModuleState optimize(SwerveModuleState desiredState, 
    double currentAngle, double currV) {
        // desired angle diff in [-360, +360]
        double _angleDiff = (desiredState.angle.getDegrees() - currentAngle) % 360;

        double targetAngle = currentAngle + _angleDiff;
        double targetSpeed = desiredState.speedMetersPerSecond;

        // Q1 undershot. We expect a CW turn.
        if (_angleDiff <= -270)
            targetAngle += 360;

        // Q2 undershot. We expect a CCW turn to Q4 & reverse direction.
        // Q3. We expect a CW turn to Q1 & reverse direction.
        else if (-90 > _angleDiff && _angleDiff > -270) {
            targetAngle += 180;
            targetSpeed = -targetSpeed;
        }

        // Q2. We expect a CCW turn to Q4 & reverse direction.
        // Q3 overshot. We expect a CW turn to Q1 & reverse direction.
        else if (90 < _angleDiff && _angleDiff < 270) {
            targetAngle -= 180;
            targetSpeed = -targetSpeed;
        }

        // Q4 overshot. We expect a CCW turn.
        else if (_angleDiff >= 270)
            targetAngle -= 360;

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }
}
