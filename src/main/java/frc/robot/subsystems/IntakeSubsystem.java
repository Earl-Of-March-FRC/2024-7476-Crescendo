// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(9);
  private final AnalogPotentiometer ultrasonic = new AnalogPotentiometer(new AnalogInput(0), 15);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ultrasonic Distance", ultrasonic.get());
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

}
