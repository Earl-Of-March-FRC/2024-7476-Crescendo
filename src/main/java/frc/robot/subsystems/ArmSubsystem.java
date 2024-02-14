// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final VictorSP leftArmMotor = new VictorSP(0);
  private final VictorSP rightArmMotor = new VictorSP(1);

  private final Encoder encoder = new Encoder();


  public ArmSubsystem() {


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
