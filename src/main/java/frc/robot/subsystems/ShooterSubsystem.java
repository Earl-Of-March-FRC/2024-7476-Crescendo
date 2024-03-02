// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final WPI_TalonSRX blueShooter = new WPI_TalonSRX(7);
  private final WPI_TalonSRX orangeShooter = new WPI_TalonSRX(8);

  public ShooterSubsystem() {

    SmartDashboard.putNumber("Shooter speed", 1.0);

    blueShooter.setNeutralMode(NeutralMode.Coast);
    orangeShooter.setNeutralMode(NeutralMode.Coast);

    blueShooter.setInverted(true);
    orangeShooter.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {

    double x = SmartDashboard.getNumber("Shooter speed", 1.0);

    // blueShooter.set(speed);
    // orangeShooter.set(speed);

    // blueShooter.set(speed*x);
    blueShooter.setVoltage(x*speed);
    orangeShooter.setVoltage(x*speed);
    // orangeShooter.set(speed*x);
  }
}
