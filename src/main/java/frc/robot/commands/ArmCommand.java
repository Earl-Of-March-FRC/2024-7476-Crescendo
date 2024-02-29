// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
  private ArmSubsystem armSub;
  private double speed;

  /** Creates a new ArmCommand. */
  public ArmCommand(ArmSubsystem armSub, double speed) {
    this.armSub = armSub;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSub.setArm(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.setArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
