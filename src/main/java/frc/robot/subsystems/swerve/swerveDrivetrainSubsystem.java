// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
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


public class swerveDrivetrainSubsystem extends SubsystemBase {
  private static swerveDrivetrainSubsystem swerve;
  
  public final static double maxVelocity = 3;
  public final static double maxAcceleration = 2;
  public final static double maxAngularVelocity = Math.PI;
  public final static double maxAngularAcceleration = 1;
  public final static double maxAngularVelocityPerModule = Math.PI;
  public final static double maxAngularAccelerationPerModule = 1;

  private final Translation2d frontLeftLocation = new Translation2d(0.35, 0.35);
  private final Translation2d frontRightLocation = new Translation2d(0.35, -0.35);
  private final Translation2d rearLeftLocation = new Translation2d(-0.35, 0.35);
  private final Translation2d rearRightLocation = new Translation2d(-0.35, -0.35);

  private final AHRS navx = new AHRS(Port.kOnboard);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
      rearLeftLocation, rearRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(navx.getAngle()));

  private final SwerveConstants swerveConstants = new SwerveConstants(5, 42, 60);

  private final SwerveModule frontLeftModule = 
    new SwerveModule(
      SwervePortMap.leftFrontDriveID, 
      SwervePortMap.leftFrontTurningID,
      SwervePortMap.leftFrontAbsoluteEncoderID, 
      false, false, 0, false, 
      swerveConstants, 
      new ProfiledPIDController(1, 1, 1, 
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)),
      new SimpleMotorFeedforward(0, 0, 0), new PIDController(0, 0, 0));

  private final SwerveModule frontRightModule = 
    new SwerveModule(
      SwervePortMap.rightFrontDriveID, 
      SwervePortMap.rightFrontTurningID,
      SwervePortMap.rightFrontAbsoluteEncoderID, 
      false, false, 0, false, 
      swerveConstants, 
      new ProfiledPIDController(1, 1, 1, 
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)),
      new SimpleMotorFeedforward(0, 0, 0), new PIDController(0, 0, 0));

  private final SwerveModule rearLeftModule = 
    new SwerveModule(
        SwervePortMap.leftBackDriveID, 
        SwervePortMap.leftBackTurningID,
        SwervePortMap.leftBackAbsoluteEncoderID, 
        false, false, 0, false, 
        swerveConstants, 
        new ProfiledPIDController(1, 1, 1, 
          new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)),
        new SimpleMotorFeedforward(0, 0, 0), new PIDController(0, 0, 0));

  private final SwerveModule rearRightModule = 
    new SwerveModule(
      SwervePortMap.rightBackDriveID, 
      SwervePortMap.rightBackTurningID,
      SwervePortMap.rightBackAbsoluteEncoderID, 
      false, false, 0, false, 
      swerveConstants, 
      new ProfiledPIDController(1, 1, 1, 
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)),
      new SimpleMotorFeedforward(0, 0, 0), new PIDController(0, 0, 0));

  /** Creates a new DrivetrainSubsystem. */
  public swerveDrivetrainSubsystem() {
    new Thread(() ->{
      try {
        Thread.sleep(1000);
        navx.reset();
      } catch (Exception e) {
      }
    }).start();
  }

  public double getYaw() {
    return Math.IEEEremainder(navx.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(getYaw());
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
    odometry.update(new Rotation2d(navx.getAngle()), frontLeftModule.getState(),
        frontRightModule.getState(), rearLeftModule.getState(), rearRightModule.getState());
  }

  public static swerveDrivetrainSubsystem getInstance() {
    if (swerve == null) {
      return new swerveDrivetrainSubsystem();
    }
    return swerve;
  }
}