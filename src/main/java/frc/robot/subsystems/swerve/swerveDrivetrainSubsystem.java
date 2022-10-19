// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.utils.controllers.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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


public class swerveDrivetrainSubsystem extends SubsystemBase {
  private static swerveDrivetrainSubsystem swerve;
  
  // TODO
  public final static double maxVelocity = 3;
  public final static double maxAcceleration = 2;
  public final static double maxAngularVelocity = Math.PI;
  public final static double maxAngularAcceleration = 1;
  public final static double maxAngularVelocityPerModule = Math.PI;
  public final static double maxAngularAccelerationPerModule = 1;
  // TODO

  public edu.wpi.first.math.controller.PIDController P_CONTROLLER_X;
  public edu.wpi.first.math.controller.PIDController P_CONTROLLER_Y;
  public ProfiledPIDController thetaProfiledPID; // degrees

  private final String KP_X = "kp_x";
  private final String KP_Y = "kp_y";
  private final String theta_KP = "theta_KP";
  private final String theta_KI = "theta_KI";
  private final String theta_KD = "theta_KD";

  private final Shuffleboard board;

  // TODO
  private final Translation2d frontLeftLocation = new Translation2d(0.35, 0.35);
  private final Translation2d frontRightLocation = new Translation2d(0.35, -0.35);
  private final Translation2d rearLeftLocation = new Translation2d(-0.35, 0.35);
  private final Translation2d rearRightLocation = new Translation2d(-0.35, -0.35);
  // TODO

  private final AHRS navx = new AHRS(Port.kOnboard);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
      rearLeftLocation, rearRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
    new Rotation2d(Math.toRadians(navx.getAngle())));

  // TODO
  private final SwerveConstants swerveConstants = new SwerveConstants(5, 1, 60);
  // TODO

  // TODO
  private final SwerveModule frontLeftModule = 
    new SwerveModule(
      "frontLeftModule",
      SwervePortMap.leftFrontDriveID, 
      SwervePortMap.leftFrontTurningID,
      false, false, 0, 
      swerveConstants, 
      new ProfiledPIDController(1, 1, 1, 
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)),
      new SimpleMotorFeedforward(0, 0, 0), new PIDController(0, 0, 0, 0, 0, -1, 1));

  private final SwerveModule frontRightModule = 
    new SwerveModule(
      "frontRightModule",
      SwervePortMap.rightFrontDriveID, 
      SwervePortMap.rightFrontTurningID, 
      false, false, 0, 
      swerveConstants, 
      new ProfiledPIDController(1, 1, 1, 
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)),
      new SimpleMotorFeedforward(0, 0, 0), new PIDController(0, 0, 0, 0, 0, -1, 1));

  private final SwerveModule rearLeftModule = 
    new SwerveModule(
        "rearLeftModule",
        SwervePortMap.leftBackDriveID, 
        SwervePortMap.leftBackTurningID,
        false, false, 0,
        swerveConstants, 
        new ProfiledPIDController(1, 1, 1, 
          new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)),
        new SimpleMotorFeedforward(0, 0, 0), new PIDController(0, 0, 0, 0, 0, -1, 1));

  private final SwerveModule rearRightModule = 
    new SwerveModule(
      "rearRightModule",
      SwervePortMap.rightBackDriveID, 
      SwervePortMap.rightBackTurningID,
      false, false, 0,
      swerveConstants, 
      new ProfiledPIDController(1, 1, 1, 
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)),
      new SimpleMotorFeedforward(0, 0, 0), new PIDController(0, 0, 0, 0, 0, -1, 1));
  // TODO

  /** Creates a new DrivetrainSubsystem. */
  public swerveDrivetrainSubsystem() {
    new Thread(() ->{
      try {
        Thread.sleep(1000);
        navx.reset();
      } catch (Exception e) {
      }
    }).start();
    this.board = new Shuffleboard("swerve");

    // TODO
    board.addNum(KP_X, 0);
    // TODO

    P_CONTROLLER_X = new edu.wpi.first.math.controller.
      PIDController(board.getNum(KP_X),0,0);
    
    // TODO
    board.addNum(KP_Y, 0);
    // TODO

    P_CONTROLLER_Y = new edu.wpi.first.math.controller.
      PIDController(board.getNum(KP_Y),0,0);
    
    // TODO
    board.addNum(theta_KP, 0);
    board.addNum(theta_KI, 0);
    board.addNum(theta_KD, 0);
    // TODO
    thetaProfiledPID = new ProfiledPIDController(
      board.getNum(theta_KP), board.getNum(theta_KI), board.getNum(theta_KD),
      new TrapezoidProfile.Constraints(
        maxAngularVelocity, maxAngularAcceleration));  
  }

  public double getYaw() {
    return Math.IEEEremainder(navx.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getYaw()));
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

  public void setModules(SwerveModuleState[] states, boolean inAutonomous) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);
    frontLeftModule.setDesiredState(states[0], inAutonomous);
    frontRightModule.setDesiredState(states[1], inAutonomous);
    rearLeftModule.setDesiredState(states[2], inAutonomous);
    rearRightModule.setDesiredState(states[3], inAutonomous);

  }

  public void drive(double x, double y, double rotation, boolean fieldRelative,
   boolean inAutonomous) {
    SwerveModuleState[] states = kinematics
        .toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, getRotation2d())
                : new ChassisSpeeds(x, y, rotation));
    setModules(states, inAutonomous);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    P_CONTROLLER_X.setP(board.getNum(KP_X));
    P_CONTROLLER_Y.setP(board.getNum(KP_Y));
    thetaProfiledPID.setPID(
      board.getNum(theta_KP), board.getNum(theta_KI), board.getNum(theta_KD));
    odometry.update(new Rotation2d(navx.getAngle()), frontLeftModule.getState(),
        frontRightModule.getState(), rearLeftModule.getState(), rearRightModule.getState());
      
    board.addString("point", "(" + getPose().getX() + "," + getPose().getY() + ")");
    board.addNum("angle in degrees", getPose().getRotation().getDegrees());
    
    /**
     * for calibration
     */
    // frontLeftModule.turningUsingPID(100);
    // frontLeftModule.driveUsingPID(3);

    // frontRightModule.turningUsingPID(100);
    // frontRightModule.driveUsingPID(3);

    // rearLeftModule.turningUsingPID(100);
    // rearLeftModule.driveUsingPID(3);

    // rearRightModule.turningUsingPID(100);
    // rearRightModule.driveUsingPID(3);

    // drive(0, 0, 
    //   thetaProfiledPID.calculate(getPose().getRotation().getDegrees(),
    //    Math.toRadians(30)),
    //   false, true);

    // drive(0, 
    //   P_CONTROLLER_Y.calculate(getPose().getTranslation().getY(), 3),
    //   0, false, true);

    // drive( P_CONTROLLER_X.calculate(getPose().getTranslation().getX(), 3), 
    //   0, 0, false, true);
  }

  public static swerveDrivetrainSubsystem getInstance() {
    if (swerve == null) {
      return new swerveDrivetrainSubsystem();
    }
    return swerve;
  }
}