// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;
import frc.robot.utils.JoystickContainer;

public class swerveJoystickCommand extends CommandBase {
  /** Creates a new swerveJoystickCommand. */
  private final SwerveDrivetrainSubsystem swerve;
  private final SlewRateLimiter rateLimiterX;
  private final SlewRateLimiter rateLimiterY;
  private final SlewRateLimiter rateLimiterTurning;
  //private final SlewRateLimiter xRateLimiter, yRateLimiter, turningRateLimiter;
  public swerveJoystickCommand() {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    rateLimiterX = new SlewRateLimiter(2.5);
    rateLimiterY = new SlewRateLimiter(2.5);
    rateLimiterTurning = new SlewRateLimiter(2.5);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = rateLimiterX.calculate(JoystickContainer.drivingJoystick.getRawAxis(
      SwerveDrivetrainSubsystem.getInstance().isXYReversed ? 1 : 0));
    double ySpeed = rateLimiterY.calculate(JoystickContainer.drivingJoystick.getRawAxis(
      SwerveDrivetrainSubsystem.getInstance().isXYReversed ? 0 : 1));
    double turningSpeed = rateLimiterTurning.calculate(
      JoystickContainer.drivingJoystick.getRawAxis(2));

    xSpeed = Math.abs(xSpeed) < 0.05 ? 0 : xSpeed;
    ySpeed = Math.abs(ySpeed) < 0.1 ? 0 : ySpeed;
    turningSpeed = Math.abs(turningSpeed) < 0.1 ? 0 : turningSpeed;

    xSpeed = xSpeed * 
      SwerveConstants.maxVelocity * 
      (SwerveDrivetrainSubsystem.getInstance().isXReversed ? -1 : 1);
    ySpeed = ySpeed * 
      SwerveConstants.maxVelocity *
      (SwerveDrivetrainSubsystem.getInstance().isYReversed ? -1 : 1);
    turningSpeed = turningSpeed * 
      SwerveConstants.maxAngularVelocity;
    
    
    swerve.drive(xSpeed, ySpeed, turningSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
