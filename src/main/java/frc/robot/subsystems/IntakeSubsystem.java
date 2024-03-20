// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(9);
  //private final Ultrasonic ultrasonic = new Ultrasonic()
  private final AnalogPotentiometer ultrasonic = new AnalogPotentiometer(new AnalogInput(1), 15);

  public IntakeSubsystem() {
    intakeMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(ultrasonic.get() < 15){

    // }

    SmartDashboard.putNumber("ultrasonic Distance", ultrasonic.get());
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }


}
