// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final WPI_TalonSRX blueShooter = new WPI_TalonSRX(7);
  private final WPI_TalonSRX orangeShooter = new WPI_TalonSRX(8);

  public ShooterSubsystem(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    blueShooter.set(speed);
    orangeShooter.set(speed);
  }
}
