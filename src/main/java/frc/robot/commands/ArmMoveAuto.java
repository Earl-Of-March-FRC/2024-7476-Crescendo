// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmMoveAuto extends ProfiledPIDCommand {
  /** Creates a new ArmTest. */
  public ArmMoveAuto(ArmSubsystem armSub, double goalAngle) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.075,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(100, 130)),
        // This should return the measurement
        () -> armSub.getArmPosition(),
        // This should return the goal (can also be a constant)
        () -> goalAngle,
        // This uses the output
        (output, setpoint) -> {
          MathUtil.clamp(output, -0.9, 0.9);
          armSub.setArmSpeed(output);
        });

        addRequirements(armSub);
        getController().setTolerance(5);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
