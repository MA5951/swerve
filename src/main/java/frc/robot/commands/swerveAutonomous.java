// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.swerveDrivetrainSubsystem;

public class swerveAutonomous extends CommandBase {
  /** Creates a new swerveAutonomous. */
  private final swerveDrivetrainSubsystem swerve;
  private final Trajectory trajectory;
  private final HolonomicDriveController driveController;
  private final Timer timer = new Timer();
  public swerveAutonomous(Pose2d firstPose2d, List<Translation2d> list, Pose2d endPose2d) {
    swerve = swerveDrivetrainSubsystem.getInstance();
    addRequirements(swerve);
    TrajectoryConfig trajectoryConfig = 
      new TrajectoryConfig(
        swerveDrivetrainSubsystem.maxVelocity, 
        swerveDrivetrainSubsystem.maxAcceleration).
        setKinematics(swerve.getKinematics());
    trajectory = TrajectoryGenerator.generateTrajectory(
      firstPose2d, list, endPose2d, trajectoryConfig);
    PIDController xPID = new PIDController(1, 1, 1);
    PIDController yPID = new PIDController(1, 1, 1);
    ProfiledPIDController thetaProfiledPID = 
      new ProfiledPIDController(1, 1, 1, 
      new TrapezoidProfile.Constraints(swerveDrivetrainSubsystem.maxAngularVelocity, 
      swerveDrivetrainSubsystem.maxAngularAcceleration));
    driveController = new HolonomicDriveController(xPID, yPID, thetaProfiledPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    swerve.resetOdometry(trajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = timer.get();
    Trajectory.State desiredState = trajectory.sample(currentTime);

    ChassisSpeeds desiredChassisSpeeds = 
      driveController.calculate(swerve.getPose(),
       desiredState,
       trajectory.getStates().get(trajectory.getStates().size() - 1)
       .poseMeters.getRotation());
    
    swerve.drive(
      desiredChassisSpeeds.vxMetersPerSecond,
      desiredChassisSpeeds.vyMetersPerSecond,
      desiredChassisSpeeds.omegaRadiansPerSecond,
      false,
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
