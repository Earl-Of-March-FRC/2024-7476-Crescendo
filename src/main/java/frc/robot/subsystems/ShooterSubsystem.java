// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX blueShooter = new WPI_TalonSRX(7);
  private final WPI_TalonSRX orangeShooter = new WPI_TalonSRX(8);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    SmartDashboard.putNumber("Shooter speed", 1.0);

    blueShooter.setInverted(true);
    orangeShooter.setInverted(true);

    setBrakeMode();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {}

  public void setBrakeMode() {
    blueShooter.setNeutralMode(NeutralMode.Brake);
    orangeShooter.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    blueShooter.setNeutralMode(NeutralMode.Coast);
    orangeShooter.setNeutralMode(NeutralMode.Coast);
  }

  public void setShooterSpeed(double speed) {

    double x = SmartDashboard.getNumber("Shooter speed", 1.0);

    blueShooter.setVoltage(x*speed);
    orangeShooter.setVoltage(x*speed);

    // blueShooter.set(speed);
    // orangeShooter.set(speed);

    // blueShooter.set(speed*x);
    // orangeShooter.set(speed*x);
  }
}
