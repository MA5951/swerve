package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.Shuffleboard;
import frc.robot.utils.controllers.PIDController;

/** Add your docs here. */
public class SwerveModule {

    // private final TalonFX driveMotor;
    // private final TalonFX turningMotor;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final ProfiledPIDController turningPID; // in degrees

    private final double offsetEncoder;

    private final PIDController drivePID;

    private final SimpleMotorFeedforward turningFeedforward;

    private final Shuffleboard board;
    private final String driveKP = "driveKP";
    private final String driveKI = "driveKI";
    private final String driveKD = "driveKD";
    private final String driveKF = "driveKF";

    private final String turningKP = "turningKP";
    private final String turningKI = "turningKI";
    private final String turningKD = "turningKD";

    public SwerveModule(String tabName,
        int driveID, int turningID, boolean isDriveMotorReversed, 
        boolean isTurningMotorReversed, double offsetEncoder,
        SwerveConstants swerveConstants, ProfiledPIDController turningPID,
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

        this.drivePID = drivePID;
        this.turningPID = turningPID;
        this.turningFeedforward = turningFeedforward;

        this.offsetEncoder = offsetEncoder;

        this.board = new Shuffleboard(tabName);

        board.addNum(driveKP, drivePID.getP());
        board.addNum(driveKI, drivePID.getI());
        board.addNum(driveKD, drivePID.getD());
        board.addNum(driveKF, drivePID.getF());

        board.addNum(turningKP, turningPID.getP());
        board.addNum(turningKI, turningPID.getI());
        board.addNum(turningKD, turningPID.getD());

        turningPID.enableContinuousInput(-180, 180);

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

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0 + offsetEncoder);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
            new Rotation2d(Math.toRadians(getTurningPosition())));
    }

    public void turningUsingPID(double setPoint) {
        turningPID.setPID(board.getNum(turningKP), board.getNum(turningKP),
            board.getNum(turningKP));
        board.addNum("Turning Position", getTurningPosition());        
        double turnOutput = turningPID.calculate(getTurningPosition(), setPoint);
        double turnFeed = turningFeedforward.calculate(turningPID.getSetpoint().velocity);
        turningMotor.set(turnOutput + turnFeed);
    }

    public void driveUsingPID(double setPoint) {
        drivePID.setPID(board.getNum(driveKP), board.getNum(driveKP),
            board.getNum(driveKP));
        drivePID.setF(board.getNum(driveKF));
        board.addNum("Drive Velocity", getDriveVelocity());
        double driveOutput = 
            drivePID.calculate(getDriveVelocity(), setPoint);
        driveMotor.set(driveOutput);
    }


    public void setDesiredState(SwerveModuleState desiredState, boolean inAutonomous) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, 
            new Rotation2d(Math.toRadians(getTurningPosition())));
        if (Math.abs(optimizedState.speedMetersPerSecond) < 0.001) { 
            stop();
        } else {
            if (inAutonomous) {
                driveUsingPID(optimizedState.speedMetersPerSecond);
            } else {
                driveMotor.set(optimizedState.speedMetersPerSecond / 
                    swerveDrivetrainSubsystem.maxVelocity);
            }
            turningUsingPID(optimizedState.angle.getRadians());
        }
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}