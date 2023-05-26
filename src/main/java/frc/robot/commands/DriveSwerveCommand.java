// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class DriveSwerveCommand extends CommandBase {
  /** Creates a new swerveJoystickCommand. */
  private final SwerveDrivetrainSubsystem swerve;
  //private final SlewRateLimiter xRateLimiter, yRateLimiter, turningRateLimiter;
  private Supplier<Double> xSpeedSupplier;
  private Supplier<Double> ySpeedSupplier;
  private Supplier<Double> turningSpeedSupplier;
  /**
   * @param xSpeedSupplier Percentage (-1 - 1)
   * @param ySpeedSupplier Percentage (-1 - 1)
   * @param turningSpeedSupplier Percentage (-1 - 1)
   */
  public DriveSwerveCommand(
    Supplier<Double> xSpeedSupplier,
    Supplier<Double> ySpeedSupplier,
    Supplier<Double> turningSpeedSupplier) {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    this.xSpeedSupplier = xSpeedSupplier;
    this.ySpeedSupplier = ySpeedSupplier;
    this.turningSpeedSupplier = turningSpeedSupplier;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = 
      SwerveDrivetrainSubsystem.getInstance().isXYReversed ? 
      ySpeedSupplier.get() : xSpeedSupplier.get();
    double ySpeed =
      SwerveDrivetrainSubsystem.getInstance().isXYReversed ? 
      xSpeedSupplier.get() : ySpeedSupplier.get();
    double turningSpeed = turningSpeedSupplier.get();

    xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : xSpeed;
    ySpeed = Math.abs(ySpeed) < 0.1 ? 0 : ySpeed;
    
    turningSpeed = (Math.abs(turningSpeed) < 0.1 ? 0 : turningSpeed) * -1;

    xSpeed = xSpeed * 
      swerve.maxVelocity * 
      (SwerveDrivetrainSubsystem.getInstance().isXReversed ? -1 : 1);
    ySpeed = ySpeed * 
      swerve.maxVelocity *
      (SwerveDrivetrainSubsystem.getInstance().isYReversed ? -1 : 1);
    turningSpeed = turningSpeed * 
      swerve.maxAngularVelocity;
    
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