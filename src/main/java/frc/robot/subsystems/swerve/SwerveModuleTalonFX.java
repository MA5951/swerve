package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.utils.Shuffleboard;

/** Add your docs here. */
public class SwerveModuleTalonFX extends SwerveModule{

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final SimpleMotorFeedforward feedforward  = 
        new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV);

    private final CANCoder absoluteEcoder;

    private final double offsetEncoder;

    private final Shuffleboard board;
    private final String driveKP = "driveKP";
    private final String driveKI = "driveKI";
    private final String driveKD = "driveKD";

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
        driveConfiguration.slot0.kF = 0;
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

    public void setNeutralMode(NeutralMode mode) {
        turningMotor.setNeutralMode(mode);
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
        turningMotor.setSelectedSensorPosition(0);
            // (getAbsoluteEncoderPosition() - offsetEncoder) /
            //             SwerveConstants.anglePerPulse);
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
        board.addNum("Drive Velocity", getDriveVelocity());
        driveMotor.set(ControlMode.Velocity,
                setPoint / SwerveConstants.distancePerPulse *
                        SwerveConstants.velocityTimeUnitInSeconds, 
                        DemandType.ArbitraryFeedForward,
                        feedforward.calculate(setPoint));
    }
}