// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.ma5951.utils.MAShuffleboard;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.RobotContainer;

public class SwerveDrivetrainSubsystem extends SubsystemBase {
  private static SwerveDrivetrainSubsystem swerve;

  public PIDController CONTROLLER_X;
  public PIDController CONTROLLER_Y;
  public PIDController thetaPID;

  public boolean isXReversed = true;
  public boolean isYReversed = true;
  public boolean isXYReversed = true;
  public double offsetAngle = 0;

  public double maxVelocity = SwerveConstants.MAX_VELOCITY;
  public double maxAngularVelocity = SwerveConstants.MAX_ANGULAR_VELOCITY;

  private static final String theta_KP = "theta_KP";
  private static final String theta_KI = "theta_KI";
  private static final String theta_KD = "theta_KD";
  
  public final MAShuffleboard board;

  private final Translation2d frontLeftLocation = new Translation2d(
      -SwerveConstants.WIDTH / 2,
      SwerveConstants.LENGTH / 2);
  private final Translation2d frontRightLocation = new Translation2d(
      SwerveConstants.WIDTH / 2,
      SwerveConstants.LENGTH / 2);
  private final Translation2d rearLeftLocation = new Translation2d(
      -SwerveConstants.WIDTH / 2,
      -SwerveConstants.LENGTH / 2);
  private final Translation2d rearRightLocation = new Translation2d(
      SwerveConstants.WIDTH / 2,
      -SwerveConstants.LENGTH / 2);

  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
      rearLeftLocation, rearRightLocation);

  private final static SwerveModule frontLeftModule = new SwerveModuleTalonFX(
      "frontLeftModule",
      PortMap.Swerve.leftFrontDriveID,
      PortMap.Swerve.leftFrontTurningID,
      PortMap.Swerve.leftFrontAbsoluteEncoder,
      SwerveConstants.FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
      SwerveConstants.FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED,
      SwerveConstants.FRONF_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
      SwerveConstants.FRONT_LEFT_MODULE_OFFSET_ENCODER);

  private final static SwerveModule frontRightModule = new SwerveModuleTalonFX(
      "frontRightModule",
      PortMap.Swerve.rightFrontDriveID,
      PortMap.Swerve.rightFrontTurningID,
      PortMap.Swerve.rightFrontAbsoluteEncoder,
      SwerveConstants.FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
      SwerveConstants.FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED,
      SwerveConstants.FRONF_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
      SwerveConstants.FRONT_RIGHT_MODULE_OFFSET_ENCODER);

  private final static SwerveModule rearLeftModule = new SwerveModuleTalonFX(
      "rearLeftModule",
      PortMap.Swerve.leftBackDriveID,
      PortMap.Swerve.leftBackTurningID,
      PortMap.Swerve.leftBackAbsoluteEncoder,
      SwerveConstants.REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
      SwerveConstants.REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED,
      SwerveConstants.REAR_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
      SwerveConstants.REAR_LEFT_MODULE_OFFSET_ENCODER);

  private final static SwerveModule rearRightModule = new SwerveModuleTalonFX(
      "rearRightModule",
      PortMap.Swerve.rightBackDriveID,
      PortMap.Swerve.rightBackTurningID,
      PortMap.Swerve.rightBackAbsoluteEncoder,
      SwerveConstants.REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
      SwerveConstants.REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED,
      SwerveConstants.REAR_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
      SwerveConstants.REAR_RIGHT_MODULE_OFFSET_ENCODER);

  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(kinematics,
    new Rotation2d(0), getSwerveModulePositions(),
    new Pose2d(0, 0, new Rotation2d(0)));
  
  private final Field2d field = 
    new Field2d();

  private static SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      rearLeftModule.getPosition(),
      frontLeftModule.getPosition(),
      rearRightModule.getPosition(),
      frontRightModule.getPosition()
    };
  }

  private static SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getState(), 
      frontRightModule.getState(),
      rearLeftModule.getState(),
      rearRightModule.getState()
    };
  }

  /** Creates a new DrivetrainSubsystem. */
  private SwerveDrivetrainSubsystem() {

    resetNavx();
   
    this.board = new MAShuffleboard("swerve");

    CONTROLLER_X = new PIDController(
      SwerveConstants.KP_X, SwerveConstants.KI_X, 0);

    CONTROLLER_Y = new PIDController(
      SwerveConstants.KP_Y, SwerveConstants.KI_Y, 0);

    board.addNum(theta_KP, SwerveConstants.THATA_KP);
    board.addNum(theta_KI, SwerveConstants.THATA_KI);
    board.addNum(theta_KD, SwerveConstants.THATA_KD);

    thetaPID = new PIDController(board.getNum(theta_KP),
     board.getNum(theta_KI), board.getNum(theta_KD));

    thetaPID.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData("Field", field);
  }

  public void setNeutralMode(NeutralMode mode) {
    frontLeftModule.setNeutralMode(mode);
    frontRightModule.setNeutralMode(mode);
    rearRightModule.setNeutralMode(mode);
    rearLeftModule.setNeutralMode(mode);
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearLeftModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  public double getAngularVelocity() {
    return this.kinematics.toChassisSpeeds(getSwerveModuleStates()).omegaRadiansPerSecond;
  }

  public double getRadialAcceleration() {
    return Math.pow(getAngularVelocity(), 2) * SwerveConstants.RADIUS;
  }

  public void updateOffset() {
    offsetAngle = getFusedHeading();
  }

  public double getFusedHeading() {
    return -navx.getAngle();
  }

  public double getRoll() {
    return navx.getRoll();
  }

  public double getPitch() {
    return navx.getPitch();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getFusedHeading()));
  }

  public void resetNavx() {
    navx.reset();
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
  }

  public void stop() {
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
  }

  public void setModules(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY);
    rearLeftModule.setDesiredState(states[0]);
    frontLeftModule.setDesiredState(states[1]);
    rearRightModule.setDesiredState(states[2]);
    frontRightModule.setDesiredState(states[3]);

  }

  public void drive(double x, double y, double omega, boolean fieldRelative) {
    SwerveModuleState[] states = kinematics
        .toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, 
            new Rotation2d(Math.toRadians((getFusedHeading() - offsetAngle))))
                : new ChassisSpeeds(x, y, omega));
    setModules(states);
  }

  public void FactorVelocityTo(double factor) {
    maxVelocity = 
      SwerveConstants.MAX_VELOCITY * factor;
    maxAngularVelocity = 
      SwerveConstants.MAX_ANGULAR_VELOCITY * factor;
  }

  public void odometrySetUpForAutonomous(PathPlannerTrajectory trajectory) {
    PathPlannerTrajectory tPathPlannerTrajectory;
    if (DriverStation.getAlliance() == Alliance.Red) {
      tPathPlannerTrajectory
       = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, Alliance.Red);
    } else {
      tPathPlannerTrajectory = trajectory;
    }
    resetNavx();
    navx.setAngleAdjustment(
      tPathPlannerTrajectory.getInitialState().holonomicRotation.getDegrees());
    Pose2d pose = new Pose2d(
      tPathPlannerTrajectory.getInitialPose().getX(),
      tPathPlannerTrajectory.getInitialPose().getY(),
      tPathPlannerTrajectory.getInitialState().holonomicRotation
    ); 
    resetOdometry(pose);
  }

  public PathPlannerTrajectory getTrajectory(String pathName) {
    return PathPlanner.loadPath(pathName, new PathConstraints(
      SwerveConstants.MAX_VELOCITY, SwerveConstants.MAX_ACCELERATION));
  }

  public Command getAutonomousPathCommand(
    String pathName, boolean isFirst) {
    PathPlannerTrajectory trajectory = getTrajectory(pathName);
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        if (isFirst) {
          odometrySetUpForAutonomous(trajectory);
        }}),
      new PPSwerveControllerCommand(
        trajectory,
        this::getPose,
        getKinematics(),
        CONTROLLER_X,
        CONTROLLER_Y,
        thetaPID,
        this::setModules,
        true,
        this),
      new InstantCommand(this::stop));
  }

  public Command getAutonomousPathCommand(
    String pathName) {
    return getAutonomousPathCommand(pathName, false);
  }

  public void updateOdometry() {
    Optional<EstimatedRobotPose> result = 
      RobotContainer.photonVision.getEstimatedRobotPose(getPose());
    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      odometry.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
      camPose.timestampSeconds);
    }
  }

  public void fixOdometry() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      navx.setAngleAdjustment((getPose().getRotation().getDegrees()) - 180);
      resetNavx();
      resetOdometry(
        new Pose2d(
          new Translation2d(
            Constants.FieldConstants.FIELD_LENGTH_METERS - getPose().getX(),
            Constants.FieldConstants.FIELD_WIDTH_METERS - getPose().getY()
          ),
          getRotation2d()
        )
      );
      offsetAngle += 180;
    }
  }

  public static SwerveDrivetrainSubsystem getInstance() {
    if (swerve == null) {
      swerve = new SwerveDrivetrainSubsystem();
    }
    return swerve;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getRotation2d(), getSwerveModulePositions());

    field.setRobotPose(getPose());

    board.addString("point", "(" + getPose().getX() + "," + getPose().getY() + ")");
    board.addNum("angle in degrees", getPose().getRotation().getDegrees());
    board.addNum("angle in radians", getPose().getRotation().getRadians());
    
  }
}