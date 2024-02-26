// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HeadSubsystem extends SubsystemBase {

  private final WPI_TalonSRX intakeRollers = new WPI_TalonSRX(9);
  private final WPI_TalonSRX blueShooter = new WPI_TalonSRX(8);
  private final WPI_TalonSRX orangeShooter = new WPI_TalonSRX(7);

  /** Creates a new HeadSubsystem. */
  public HeadSubsystem() {
    intakeRollers.setNeutralMode(NeutralMode.Coast);
    blueShooter.setNeutralMode(NeutralMode.Coast);
    orangeShooter.setNeutralMode(NeutralMode.Coast);
    orangeShooter.follow(blueShooter);
  }

  public void setRollersSpeed(double speed) {
    intakeRollers.set(speed);
  }

  public void stopRollers() {
    intakeRollers.set(0);
  }

  public void setShooterSpeed(double speed) {
    blueShooter.set(speed);
  }

  public void stopShooter() {
    blueShooter.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
