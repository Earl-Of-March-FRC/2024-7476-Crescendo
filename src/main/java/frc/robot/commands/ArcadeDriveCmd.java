// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ArcadeDriveCmd extends Command {
  /** Creates a new ArcadeDriveCmd. */
  DrivetrainSubsystem drive;
  DoubleSupplier throttle, turn;
  public ArcadeDriveCmd(DrivetrainSubsystem drive, DoubleSupplier throttle, DoubleSupplier turn) {
    this.drive = drive;
    this.throttle = throttle;
    this.turn = turn;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(throttle, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
