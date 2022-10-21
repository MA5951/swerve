// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Shuffleboard;


public class SwerveDrivetrainSubsystem extends SubsystemBase {
  private static SwerveDrivetrainSubsystem swerve;
  
  public edu.wpi.first.math.controller.PIDController P_CONTROLLER_X;
  public edu.wpi.first.math.controller.PIDController P_CONTROLLER_Y;
  public ProfiledPIDController thetaProfiledPID; // degrees

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

  private final AHRS navx = new AHRS(Port.kOnboard);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
      rearLeftLocation, rearRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
    new Rotation2d(Math.toRadians(navx.getFusedHeading())));

  private final SwerveModule frontLeftModule = 
    new SwerveModule(
      "frontLeftModule",
      SwervePortMap.leftFrontDriveID, 
      SwervePortMap.leftFrontTurningID,
      SwerveConstants.frontLeftModuleIsDriveMotorReversed,
      SwerveConstants.frontLeftModuleIsTurningMotorReversed,
      SwerveConstants.frontLeftModuleOffsetEncoder);

  private final SwerveModule frontRightModule = 
    new SwerveModule(
      "frontRightModule",
      SwervePortMap.rightFrontDriveID, 
      SwervePortMap.rightFrontTurningID, 
      SwerveConstants.frontRightModuleIsDriveMotorReversed,
      SwerveConstants.frontRightModuleIsTurningMotorReversed,
      SwerveConstants.frontRightModuleOffsetEncoder);

  private final SwerveModule rearLeftModule = 
    new SwerveModule(
        "rearLeftModule",
        SwervePortMap.leftBackDriveID, 
        SwervePortMap.leftBackTurningID,
        SwerveConstants.rearLeftModuleIsDriveMotorReversed,
        SwerveConstants.rearLeftModuleIsTurningMotorReversed,
        SwerveConstants.rearLeftModuleOffsetEncoder);

  private final SwerveModule rearRightModule = 
    new SwerveModule(
      "rearRightModule",
      SwervePortMap.rightBackDriveID, 
      SwervePortMap.rightBackTurningID,
      SwerveConstants.rearRightModuleIsDriveMotorReversed,
      SwerveConstants.rearRightModuleIsTurningMotorReversed,
      SwerveConstants.rearRightModuleOffsetEncoder);

  /** Creates a new DrivetrainSubsystem. */
  public SwerveDrivetrainSubsystem() {
    new Thread(() ->{
      try {
        Thread.sleep(1000);
        navx.reset();
      } catch (Exception e) {
      }
    }).start();
    this.board = new Shuffleboard("swerve");

    board.addNum(KP_X, SwerveConstants.KP_X);

    P_CONTROLLER_X = new edu.wpi.first.math.controller.
      PIDController(board.getNum(KP_X),0,0);
    
    board.addNum(KP_Y, SwerveConstants.KP_Y);

    P_CONTROLLER_Y = new edu.wpi.first.math.controller.
      PIDController(board.getNum(KP_Y),0,0);
    
    board.addNum(theta_KP, SwerveConstants.theta_KP);
    board.addNum(theta_KI, SwerveConstants.theta_KI);
    board.addNum(theta_KD, SwerveConstants.theta_KD);

    thetaProfiledPID = new ProfiledPIDController(
      board.getNum(theta_KP), board.getNum(theta_KI), board.getNum(theta_KD),
      new TrapezoidProfile.Constraints(
        SwerveConstants.maxAngularVelocity, SwerveConstants.maxAngularAcceleration));  
  }

  public double getYaw() {
    return Math.IEEEremainder(navx.getFusedHeading(), 360);
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getYaw()));
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
    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    rearLeftModule.setDesiredState(states[2]);
    rearRightModule.setDesiredState(states[3]);
  }

  public void drive(double x, double y, double rotation, boolean fieldRelative) {
    SwerveModuleState[] states = kinematics
        .toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getRotation2d())
                : new ChassisSpeeds(x, y, rotation));
    setModules(states);
  }

  public static SwerveDrivetrainSubsystem getInstance() {
    if (swerve == null) {
      swerve =  new SwerveDrivetrainSubsystem();
    }
    return swerve;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    P_CONTROLLER_X.setP(board.getNum(KP_X));
    P_CONTROLLER_Y.setP(board.getNum(KP_Y));
    thetaProfiledPID.setPID(
      board.getNum(theta_KP), board.getNum(theta_KI), board.getNum(theta_KD));
    odometry.update(new Rotation2d(Math.toRadians(navx.getFusedHeading())), frontLeftModule.getState(),
        frontRightModule.getState(), rearLeftModule.getState(), rearRightModule.getState());
      
    board.addString("point", "(" + getPose().getX() + "," + getPose().getY() + ")");
    board.addNum("angle in degrees", getPose().getRotation().getDegrees());
    
    /**
     * for calibration
     */
    // frontLeftModule.turningMotorSetPower(0.3);
    // frontLeftModule.driveMotorSetPower(0.3);
    // frontLeftModule.turningUsingPID(100);
    // frontLeftModule.driveUsingPID(3);

    // frontRightModule.turningMotorSetPower(0.3);
    // frontRightModule.driveMotorSetPower(0.3);
    // frontRightModule.turningUsingPID(100);
    // frontRightModule.driveUsingPID(3);

    // rearLeftModule.turningMotorSetPower(0.3);
    // rearLeftModule.driveMotorSetPower(0.3);
    // rearLeftModule.turningUsingPID(100);
    // rearLeftModule.driveUsingPID(3);

    // rearRightModule.turningMotorSetPower(0.3);
    // rearRightModule.driveMotorSetPower(0.3);
    // rearRightModule.turningUsingPID(100);
    // rearRightModule.driveUsingPID(3);

    // drive(0, 0, 
    //   thetaProfiledPID.calculate(getPose().getRotation().getDegrees(),
    //   30), false);

    // drive(0, 
    //   P_CONTROLLER_Y.calculate(getPose().getTranslation().getY(), 3),
    //   0, false);

    // drive( P_CONTROLLER_X.calculate(getPose().getTranslation().getX(), 3), 
    //   0, 0, false);
  }
}