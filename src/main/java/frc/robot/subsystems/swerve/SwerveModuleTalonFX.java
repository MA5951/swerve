package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class SwerveModuleTalonFX extends SwerveModule{

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final SimpleMotorFeedforward feedforward  = 
        new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV);

    private final CANCoder absoluteEcoder;

    private final boolean isAbsoluteEncoderReversed;
    private final double offsetEncoder;

    public SwerveModuleTalonFX(String tabName, int driveID,
            int turningID, int absoluteEncoderID, boolean isDriveMotorReversed,
            boolean isTurningMotorReversed, boolean isAbsoluteEncoderReversed,
            double offsetEncoder) {
        this.absoluteEcoder = new CANCoder(absoluteEncoderID);

        this.offsetEncoder = offsetEncoder;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        this.driveMotor = new TalonFX(driveID);
        this.turningMotor = new TalonFX(turningID);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(isDriveMotorReversed);
        turningMotor.setInverted(isTurningMotorReversed);

        configTurningMotor();
        configDriveMotor();
        resetEncoders();

        absoluteEcoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 15000);
        absoluteEcoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 15000);
    }

    private void configTurningMotor() {
        TalonFXConfiguration turningConfiguration = new TalonFXConfiguration();

        turningConfiguration.slot0.kP = SwerveConstants.TURNING_PIDKP;
        turningConfiguration.slot0.kI = SwerveConstants.TURNING_PIDKI;
        turningConfiguration.slot0.kD = SwerveConstants.TURNING_PIDKD;
        turningConfiguration.slot0.kF = 0;
        turningConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                SwerveConstants.TURING_ENABLE_CURRENT_LIMIT,
                SwerveConstants.TURNING_CONTINUOUS_CURRENT_LIMIT,
                SwerveConstants.TURNING_PEAK_CURRENT_LIMIT,
                SwerveConstants.TURNING_PEAK_CURRENT_DURATION);
        turningConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;
        turningMotor.configAllSettings(turningConfiguration);
    }

    private void configDriveMotor() {
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();

        driveConfiguration.slot0.kP = SwerveConstants.DRIVE_PID_KP;
        driveConfiguration.slot0.kI = SwerveConstants.DRIVE_PID_KI;
        driveConfiguration.slot0.kD = SwerveConstants.DRIVE_PID_KD;
        driveConfiguration.slot0.kF = 0;
        driveConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                SwerveConstants.DRIVE_ENBLE_CURRENT_LIMIT,
                SwerveConstants.DRIVE_CONTINUOS_CURRENT_LIMIT,
                SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT,
                SwerveConstants.DRIVE_PEAK_CURRENT_DURATION);
        driveConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;
        driveConfiguration.openloopRamp = SwerveConstants.OPEN_LOOP_RAMP;
        driveConfiguration.closedloopRamp = SwerveConstants.CLOSED_LOOP_RAMP;

        driveMotor.configAllSettings(driveConfiguration);
    }

    public void setNeutralMode(NeutralMode mode) {
        turningMotor.setNeutralMode(mode);
    }

    public double getAbsoluteEncoderPosition() {
        return isAbsoluteEncoderReversed ?
         360 - absoluteEcoder.getAbsolutePosition() :
         absoluteEcoder.getAbsolutePosition();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition()
                * SwerveConstants.DISTANCE_PER_PULSE;
    }

    public double getTurningPosition() {
        return turningMotor.getSelectedSensorPosition()
                * SwerveConstants.ANGLE_PER_PULSE;
    }

    public double getDriveVelocity() {
        return (driveMotor.getSelectedSensorVelocity()
                * SwerveConstants.DISTANCE_PER_PULSE) /
                SwerveConstants.VELOCITY_TIME_UNIT_IN_SECONDS;
    }

    public double getTurningVelocity() {
        return (turningMotor.getSelectedSensorVelocity()
                * SwerveConstants.ANGLE_PER_PULSE) /
                SwerveConstants.VELOCITY_TIME_UNIT_IN_SECONDS;
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(
            (getAbsoluteEncoderPosition() - offsetEncoder) /
                        SwerveConstants.ANGLE_PER_PULSE);
    }

    public void turningMotorSetPower(double power) {
        turningMotor.set(ControlMode.PercentOutput, power);
    }

    public void driveMotorSetPower(double power) {
        driveMotor.set(ControlMode.PercentOutput, power);
    }

    public void setInvertedTurning(Boolean turningMode) {
        turningMotor.setInverted(turningMode);
    }

    public void turningUsingPID(double setPoint) {
        turningMotor.set(ControlMode.Position, setPoint /
                SwerveConstants.ANGLE_PER_PULSE);
    }

    public void driveUsingPID(double setPoint) {
        driveMotor.set(ControlMode.Velocity,
                setPoint / SwerveConstants.DISTANCE_PER_PULSE *
                        SwerveConstants.VELOCITY_TIME_UNIT_IN_SECONDS, 
                        DemandType.ArbitraryFeedForward,
                        feedforward.calculate(setPoint));
    }
}