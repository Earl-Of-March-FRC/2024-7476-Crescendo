// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Colours;;


public class SetLEDColour extends Command {
  /** Creates a new SetLEDColour. */

  private LEDSubsystem ledSub;
  private Colours colour;
  public SetLEDColour(LEDSubsystem ledSub, Colours colour) {
    this.colour = colour;
    this.ledSub = ledSub;
    addRequirements(ledSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledSub.setLEDColour(colour);
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
