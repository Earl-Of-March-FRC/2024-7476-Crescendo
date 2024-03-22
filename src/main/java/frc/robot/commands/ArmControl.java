// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;

// led count 151

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmControl extends SequentialCommandGroup {
  /** Creates a new ArmControl. */
  public ArmControl(ArmSubsystem armsub, DoubleSupplier speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new ArmMoveManual(armsub, speed).onlyWhile(() -> armsub.getArmPosition()<110),
      new ArmPID(armsub, 105).onlyIf(() -> (armsub.getArmPosition()>110 && SmartDashboard.getBoolean("Soft Limit", true)))
      );
    addRequirements(armsub);

    // addCommands(new ArmPID(armsub, 102).onlyIf( () -> armsub.getArmPosition()>110));
  }
}
