// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class testP extends ProfiledPIDCommand {
  /** Creates a new testP. */
  public testP(DrivetrainSubsystem drive) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            DrivetrainConstants.P,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1, 2)),
        // This should return the measurement
        () -> drive.getLeftDistance(),
        // This should return the goal (can also be a constant)
        () -> 2,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drive.tankDrive(()-> output, () -> output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drive);
    getController().setTolerance(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(getController().atSetpoint()){
      return true;
    }
    return false;
  }
}
