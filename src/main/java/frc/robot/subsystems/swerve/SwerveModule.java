package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.Shuffleboard;

/** Add your docs here. */
public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPID; // in degrees

    private final double offsetEncoder;

    private final PIDController drivePID;

    private final CANCoder absoluteEcoder;

    private final Shuffleboard board;
    private final String driveKP = "driveKP";
    private final String driveKI = "driveKI";
    private final String driveKD = "driveKD";

    private final String turningKP = "turningKP";
    private final String turningKI = "turningKI";
    private final String turningKD = "turningKD";

    public SwerveModule(String tabName, int driveID, 
        int turningID, int absoluteEcoderID, boolean isDriveMotorReversed,
        boolean isTurningMotorReversed, double offsetEncoder) {
        this.driveMotor = new TalonFX(driveID);
        this.turningMotor = new TalonFX(turningID);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(isDriveMotorReversed);
        turningMotor.setInverted(isTurningMotorReversed);

        this.absoluteEcoder = new CANCoder(absoluteEcoderID);

        this.drivePID = new PIDController(
            SwerveConstants.drivePIDKP, 
            SwerveConstants.drivePIDKI,
            SwerveConstants.drivePIDKD);
        
        this.turningPID = new PIDController(
            SwerveConstants.turningPIDKP,
            SwerveConstants.turningPIDKI,
            SwerveConstants.turningPIDKD);

        this.offsetEncoder = offsetEncoder;

        this.board = new Shuffleboard(tabName);

        board.addNum(driveKP, drivePID.getP());
        board.addNum(driveKI, drivePID.getI());
        board.addNum(driveKD, drivePID.getD());

        board.addNum(turningKP, turningPID.getP());
        board.addNum(turningKI, turningPID.getI());
        board.addNum(turningKD, turningPID.getD());
    }

    public double getabsoluteEcoderPosition() {
        return ((absoluteEcoder.getPosition() / 
            SwerveConstants.CANcoderResolution) * 360) % 360; 
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition() 
        * SwerveConstants.distancePerPulse;
    }

    public double getTurningPosition() {
        return turningMotor.getSelectedSensorPosition()
        * SwerveConstants.anglePerPulse;
    }

    public double getDriveVelocity() {
        return (driveMotor.getSelectedSensorVelocity()
        * SwerveConstants.distancePerPulse) / 
        SwerveConstants.velocityTimeUnitInSeconds;
    }

    public double getTurningVelocity() {
        return (turningMotor.getSelectedSensorVelocity()
        * SwerveConstants.anglePerPulse) /
        SwerveConstants.velocityTimeUnitInSeconds;
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(
            (getabsoluteEcoderPosition() + offsetEncoder) / 
            SwerveConstants.anglePerPulse);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
            new Rotation2d(Math.toRadians(getTurningPosition())));
    }

    public void turningMotorSetPower(double power) {
        turningMotor.set(ControlMode.PercentOutput, power);
    }

    public void driveMotorSetPower(double power) {
        driveMotor.set(ControlMode.PercentOutput, power);
    }

    public void turningUsingPID(double setPoint) {
        turningPID.setPID(board.getNum(turningKP), board.getNum(turningKI),
            board.getNum(turningKD));
        board.addNum("Turning Position", getTurningPosition());        
        double turnOutput = turningPID.calculate(getTurningPosition(), setPoint);
        turningMotorSetPower(turnOutput);
    }

    public void driveUsingPID(double setPoint) {
        drivePID.setPID(board.getNum(driveKP), board.getNum(driveKI),
            board.getNum(driveKD));
        double F = setPoint / SwerveConstants.maxVelocity;
        board.addNum("Drive Velocity", getDriveVelocity());
        double driveOutput =
            drivePID.calculate(getDriveVelocity(), setPoint) + F;
        driveMotorSetPower(driveOutput);
    }


    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModule.optimize(desiredState, 
           getTurningPosition());
        driveUsingPID(optimizedState.speedMetersPerSecond);
        if (optimizedState.speedMetersPerSecond != 0) {
            turningUsingPID(optimizedState.angle.getDegrees());
        } else {
            turningMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
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

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}