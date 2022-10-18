package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class SwerveModule {

    // private final TalonFX driveMotor;
    // private final TalonFX turningMotor;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final ProfiledPIDController turningPID;

    private final AnalogInput absoluteEncoder;
    private final boolean isAbsoluteEncoderReversed;
    private final double absoluteEncoderOffSet;

    private final PIDController drivePID;

    private final SimpleMotorFeedforward turningFeedforward;

    public SwerveModule(int driveID, int turningID, int absoluteEncoderID, 
        boolean isDriveMotorReversed, boolean isTurningMotorReversed, double absoluteEncoderOffSet,
        boolean isAbsoluteEncoderReversed, SwerveConstants swerveConstants, ProfiledPIDController turningPID,
         SimpleMotorFeedforward turningFeedforward, PIDController drivePID) {
        // this.driveMotor = new TalonFX(driveID);
        // this.turningMotor = new TalonFX(turningID);
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningID, MotorType.kBrushless);

        driveMotor.setInverted(isDriveMotorReversed);
        turningMotor.setInverted(isTurningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(swerveConstants.getDistancePerPulse());
        driveEncoder.setVelocityConversionFactor(swerveConstants.getDistancePerPulse() 
        / swerveConstants.getVelocityTimeUnitInSeconds());
        turningEncoder.setPositionConversionFactor(swerveConstants.getAnglePerPulse());
        turningEncoder.setVelocityConversionFactor(swerveConstants.getAnglePerPulse() 
        / swerveConstants.getVelocityTimeUnitInSeconds());

        this.absoluteEncoderOffSet = absoluteEncoderOffSet;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderID);

        this.drivePID = drivePID;
        this.turningPID = turningPID;
        this.turningFeedforward = turningFeedforward;

        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoder() {
        return 
            (((absoluteEncoder.getVoltage() / RobotController.getVoltage5V())* 2 * Math.PI) 
            - absoluteEncoderOffSet) * (isAbsoluteEncoderReversed ? -1 : 1);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoder());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean inAutonomous) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, 
            new Rotation2d(getTurningPosition()));
        double driveOutput;
        if (inAutonomous) {
            driveOutput = drivePID.calculate(getDriveVelocity(), optimizedState.speedMetersPerSecond);
        } else {
            driveOutput = optimizedState.speedMetersPerSecond / swerveDrivetrainSubsystem.maxVelocity;
        }
        double turnOutput = turningPID.calculate(getTurningPosition(), optimizedState.angle.getRadians());
        double turnFeed = turningFeedforward.calculate(turningPID.getSetpoint().velocity);

        if (Math.abs(optimizedState.speedMetersPerSecond) < 0.001) {
            driveOutput = 0;
            turnOutput = 0;
            turnFeed = 0;
        }

        driveMotor.set(driveOutput);
        turningMotor.set(turnOutput + turnFeed);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}