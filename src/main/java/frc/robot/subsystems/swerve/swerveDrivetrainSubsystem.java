// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Shuffleboard;

public class SwerveDrivetrainSubsystem extends SubsystemBase {
  private static SwerveDrivetrainSubsystem swerve;

  public PIDController P_CONTROLLER_X;
  public PIDController P_CONTROLLER_Y;
  public PIDController thetaPID;

  private final String KP_X = "kp_x";
  private final String KP_Y = "kp_y";
  private final String theta_KP = "theta_KP";
  private final String theta_KI = "theta_KI";
  private final String theta_KD = "theta_KD";

  private final Shuffleboard board;

  private final Translation2d frontLeftLocation = new Translation2d(
      -SwerveConstants.width / 2,
      SwerveConstants.length / 2);
  private final Translation2d frontRightLocation = new Translation2d(
      SwerveConstants.width / 2,
      SwerveConstants.length / 2);
  private final Translation2d rearLeftLocation = new Translation2d(
      -SwerveConstants.width / 2,
      -SwerveConstants.length / 2);
  private final Translation2d rearRightLocation = new Translation2d(
      SwerveConstants.width / 2,
      -SwerveConstants.length / 2);

  private final AHRS navx = new AHRS(Port.kMXP);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
      rearLeftLocation, rearRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
      new Rotation2d(Math.toRadians(navx.getFusedHeading())));

  private final SwerveModule frontLeftModule = new SwerveModuleTalonFX(
      "frontLeftModule",
      SwervePortMap.leftFrontDriveID,
      SwervePortMap.leftFrontTurningID,
      SwervePortMap.leftFrontAbsoluteEcoder,
      SwerveConstants.frontLeftModuleIsDriveMotorReversed,
      SwerveConstants.frontLeftModuleIsTurningMotorReversed,
      SwerveConstants.frontLeftModuleOffsetEncoder);

  private final SwerveModule frontRightModule = new SwerveModuleTalonFX(
      "frontRightModule",
      SwervePortMap.rightFrontDriveID,
      SwervePortMap.rightFrontTurningID,
      SwervePortMap.rightFrontAbsoluteEcoder,
      SwerveConstants.frontRightModuleIsDriveMotorReversed,
      SwerveConstants.frontRightModuleIsTurningMotorReversed,
      SwerveConstants.frontRightModuleOffsetEncoder);

  private final SwerveModule rearLeftModule = new SwerveModuleTalonFX(
      "rearLeftModule",
      SwervePortMap.leftBackDriveID,
      SwervePortMap.leftBackTurningID,
      SwervePortMap.leftBackAbsoluteEcoder,
      SwerveConstants.rearLeftModuleIsDriveMotorReversed,
      SwerveConstants.rearLeftModuleIsTurningMotorReversed,
      SwerveConstants.rearLeftModuleOffsetEncoder);

  private final SwerveModule rearRightModule = new SwerveModuleTalonFX(
      "rearRightModule",
      SwervePortMap.rightBackDriveID,
      SwervePortMap.rightBackTurningID,
      SwervePortMap.rightBackAbsoluteEcoder,
      SwerveConstants.rearRightModuleIsDriveMotorReversed,
      SwerveConstants.rearRightModuleIsTurningMotorReversed,
      SwerveConstants.rearRightModuleOffsetEncoder);

  /** Creates a new DrivetrainSubsystem. */
  public SwerveDrivetrainSubsystem() {
    resetNavx();
    this.board = new Shuffleboard("swerve");

    board.addNum(KP_X, SwerveConstants.KP_X);

    P_CONTROLLER_X = new PIDController(board.getNum(KP_X), 0, 0);

    board.addNum(KP_Y, SwerveConstants.KP_Y);

    P_CONTROLLER_Y = new PIDController(board.getNum(KP_Y), 0, 0);

    board.addNum(theta_KP, SwerveConstants.theta_KP);
    board.addNum(theta_KI, SwerveConstants.theta_KI);
    board.addNum(theta_KD, SwerveConstants.theta_KD);

    thetaPID = new PIDController(board.getNum(theta_KP),
     board.getNum(theta_KI), board.getNum(theta_KD));
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

  public double getFusedHeading() {
    return navx.getFusedHeading();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getFusedHeading()));
  }

  public void resetNavx() {
    navx.reset();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getRotation2d());
  }

  public void stop() {
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
  }

  public void setModules(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxVelocity);
    frontLeftModule.setDesiredState(states[3]);
    frontRightModule.setDesiredState(states[1]);
    rearLeftModule.setDesiredState(states[2]);
    rearRightModule.setDesiredState(states[0]);
  }

  public void drive(double x, double y, double omega, boolean fieldRelative) {
    SwerveModuleState[] states = kinematics
        .toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, getRotation2d())
                : new ChassisSpeeds(x, y, omega));
    board.addNum("fld", states[3].angle.getDegrees());
    board.addNum("frd", states[1].angle.getDegrees());
    board.addNum("rld", states[2].angle.getDegrees());
    board.addNum("rrd", states[0].angle.getDegrees());
    setModules(states);
  }

  public static SwerveDrivetrainSubsystem getInstance() {
    if (swerve == null) {
      swerve = new SwerveDrivetrainSubsystem();
    }
    return swerve;
  }

  public Command getAutonomousPathCommand(
    String pathName, boolean isFirst, Command[] events) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName,
     new PathConstraints(
      SwerveConstants.maxVelocity, SwerveConstants.maxAcceleration));
    HashMap<String, Command> eventsMap = new HashMap<String, Command>();
    for (int i = 0; i < events.length; i++) {
      eventsMap.put("marker" + (i + 1), events[i]);
    }


    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        if (isFirst) {
          resetOdometry(trajectory.getInitialPose());
        }
      }),
      new PPSwerveControllerCommand(
        trajectory,
        this::getPose,
        getKinematics(),
        P_CONTROLLER_X,
        P_CONTROLLER_Y,
        thetaPID,
        this::setModules,
        eventsMap,
        this)
    );
  }

  public Command getAutonomousPathCommand(
    String pathName, boolean isFirst) {
    return getAutonomousPathCommand(pathName, isFirst, new Command[0]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    P_CONTROLLER_X.setP(board.getNum(KP_X));
    P_CONTROLLER_Y.setP(board.getNum(KP_Y));
    thetaPID.setPID(board.getNum(theta_KP), board.getNum(theta_KI), board.getNum(theta_KD));
    odometry.update(new Rotation2d(Math.toRadians(getFusedHeading())), frontLeftModule.getState(),
        frontRightModule.getState(), rearLeftModule.getState(), rearRightModule.getState());

    board.addString("point", "(" + getPose().getX() + "," + getPose().getY() + ")");
    board.addNum("angle in degrees", getPose().getRotation().getDegrees());
    board.addNum("angle in radians", getPose().getRotation().getRadians());

    board.addNum("cancoder frontLeft", frontLeftModule.getAbsoluteEncoderPosition());
    board.addNum("cancoder frontRight", frontRightModule.getAbsoluteEncoderPosition());
    board.addNum("cancoder rearLeft", rearLeftModule.getAbsoluteEncoderPosition());
    board.addNum("cancoder rearRight", rearRightModule.getAbsoluteEncoderPosition());

    board.addNum("frontLeft angle", frontLeftModule.getTurningPosition());
    board.addNum("frontRight angle", frontRightModule.getTurningPosition());
    board.addNum("rearLeft angle", rearLeftModule.getTurningPosition());
    board.addNum("rearRight angle", rearRightModule.getTurningPosition());

    board.addNum("frontLeft drive pose", frontLeftModule.getDrivePosition());
    board.addNum("rearLeft drive pose", rearLeftModule.getDrivePosition());
    board.addNum("frontRight drive pose", frontRightModule.getDrivePosition());
    board.addNum("rearRight drive pose", rearRightModule.getDrivePosition());
  }
}