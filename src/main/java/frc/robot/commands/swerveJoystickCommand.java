// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.swerveDrivetrainSubsystem;
import frc.robot.utils.JoystickContainer;

public class swerveJoystickCommand extends CommandBase {
  /** Creates a new swerveJoystickCommand. */
  private final swerveDrivetrainSubsystem swerve;
  //private final SlewRateLimiter xRateLimiter, yRateLimiter, turningRateLimiter;
  public swerveJoystickCommand() {
    // xRateLimiter = new SlewRateLimiter(swerveDrivetrainSubsystem.maxAcceleration);
    // yRateLimiter = new SlewRateLimiter(swerveDrivetrainSubsystem.maxAcceleration);
    // turningRateLimiter = new SlewRateLimiter(swerveDrivetrainSubsystem.maxAngularAcceleration);
    swerve = swerveDrivetrainSubsystem.getInstance();
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = JoystickContainer.drivingJoystick.getLeftX();
    double ySpeed = JoystickContainer.drivingJoystick.getLeftY();
    double turningSpeed = JoystickContainer.drivingJoystick.getRightX();

    xSpeed = Math.abs(xSpeed) < 0.01 ? 0 : xSpeed;
    ySpeed = Math.abs(xSpeed) < 0.01 ? 0 : ySpeed;
    turningSpeed = Math.abs(xSpeed) < 0.01 ? 0 : turningSpeed;

    xSpeed = xSpeed * 
    swerveDrivetrainSubsystem.maxVelocity / 4;
    ySpeed = ySpeed * 
    swerveDrivetrainSubsystem.maxVelocity / 4;
    turningSpeed = turningSpeed * 
      swerveDrivetrainSubsystem.maxVelocity / 4;
    
    swerve.drive(xSpeed, ySpeed, turningSpeed, true, false);
    


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
