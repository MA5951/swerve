package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.Shuffleboard;

/** Add your docs here. */
public class SwerveModuleTalonFX implements SwerveModule{

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final CANCoder absoluteEcoder;

    private final double offsetEncoder;

    private final Shuffleboard board;
    private final String driveKP = "driveKP";
    private final String driveKI = "driveKI";
    private final String driveKD = "driveKD";
    private final String driveKF = "driveKF";

    private final String turningKP = "turningKP";
    private final String turningKI = "turningKI";
    private final String turningKD = "turningKD";

    public SwerveModuleTalonFX(String tabName, int driveID,
            int turningID, int absoluteEcoderID, boolean isDriveMotorReversed,
            boolean isTurningMotorReversed, double offsetEncoder) {
        this.absoluteEcoder = new CANCoder(absoluteEcoderID);

        this.offsetEncoder = offsetEncoder;

        this.board = new Shuffleboard(tabName);

        this.driveMotor = new TalonFX(driveID);
        this.turningMotor = new TalonFX(turningID);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(isDriveMotorReversed);
        turningMotor.setInverted(isTurningMotorReversed);

        configTurningMotor();
        configDriveMotor();
        resetEncoders();

        board.addNum(driveKP, SwerveConstants.drivePIDKP);
        board.addNum(driveKI, SwerveConstants.drivePIDKI);
        board.addNum(driveKD, SwerveConstants.drivePIDKD);
        board.addNum(driveKF, SwerveConstants.drivePIDKF);

        board.addNum(turningKP, SwerveConstants.turningPIDKP);
        board.addNum(turningKI, SwerveConstants.turningPIDKI);
        board.addNum(turningKD, SwerveConstants.turningPIDKD);
    }

    private void configTurningMotor() {
        TalonFXConfiguration turningConfiguration = new TalonFXConfiguration();

        turningConfiguration.slot0.kP = SwerveConstants.turningPIDKP;
        turningConfiguration.slot0.kI = SwerveConstants.turningPIDKI;
        turningConfiguration.slot0.kD = SwerveConstants.turningPIDKD;
        turningConfiguration.slot0.kF = 0;
        turningConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                SwerveConstants.turningEnableCurrentLimit,
                SwerveConstants.turningContinuousCurrentLimit,
                SwerveConstants.turningPeakCurrentLimit,
                SwerveConstants.turningPeakCurrentDuration);
        turningConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;
        turningMotor.configAllSettings(turningConfiguration);
    }

    private void configDriveMotor() {
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();

        driveConfiguration.slot0.kP = SwerveConstants.drivePIDKP;
        driveConfiguration.slot0.kI = SwerveConstants.drivePIDKI;
        driveConfiguration.slot0.kD = SwerveConstants.drivePIDKD;
        driveConfiguration.slot0.kF = SwerveConstants.drivePIDKF;
        driveConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                SwerveConstants.driveEnableCurrentLimit,
                SwerveConstants.driveContinuousCurrentLimit,
                SwerveConstants.drivePeakCurrentLimit,
                SwerveConstants.drivePeakCurrentDuration);
        driveConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;
        driveConfiguration.openloopRamp = SwerveConstants.openloopRamp;
        driveConfiguration.closedloopRamp = SwerveConstants.closedloopRamp;

        driveMotor.configAllSettings(driveConfiguration);
    }

    public double getAbsoluteEncoderPosition() {
        return absoluteEcoder.getAbsolutePosition();
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
                (getAbsoluteEncoderPosition() - offsetEncoder) /
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
        turningMotor.config_kP(0, board.getNum(turningKP));
        turningMotor.config_kI(0, board.getNum(turningKI));
        turningMotor.config_kD(0, board.getNum(turningKD));
        board.addNum("Turning Position", getTurningPosition());
        turningMotor.set(ControlMode.Position, setPoint /
                SwerveConstants.anglePerPulse);
    }

    public void driveUsingPID(double setPoint) {
        driveMotor.config_kP(0, board.getNum(driveKP));
        driveMotor.config_kI(0, board.getNum(driveKI));
        driveMotor.config_kD(0, board.getNum(driveKD));
        driveMotor.config_kF(0, board.getNum(driveKF));
        board.addNum("Drive Velocity", getDriveVelocity());
        driveMotor.set(ControlMode.Velocity,
                setPoint / SwerveConstants.distancePerPulse *
                        SwerveConstants.velocityTimeUnitInSeconds);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleTalonFX.optimize(desiredState,
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