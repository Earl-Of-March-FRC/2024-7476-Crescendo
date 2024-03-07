// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMoveManual extends Command {
  private ArmSubsystem armSub;
  private DoubleSupplier speed;

  // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
  SlewRateLimiter filter = new SlewRateLimiter(0.9);

  /** Creates a new ArmCommand. */
  public ArmMoveManual(ArmSubsystem armSub, DoubleSupplier d) {
    this.armSub = armSub;
    this.speed = d;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(speed.getAsDouble()) < 0.3){ armSub.setArmSpeed(speed.getAsDouble());}
    else{armSub.setArmSpeed(filter.calculate(speed.getAsDouble())*0.7);}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
