// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CancelDriveAutos extends InstantCommand {
  

  Command toSource;
  Command toAmp;
  public CancelDriveAutos(Command toSource, Command toAmp) {
    
    this.toSource = toSource;
    this.toAmp = toAmp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toSource.cancel();
    toAmp.cancel();
  }
}
