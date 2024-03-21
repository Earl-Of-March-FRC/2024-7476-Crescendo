// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;


public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final WPI_TalonSRX leftArmMotor = new WPI_TalonSRX(6);
  private final WPI_TalonSRX rightArmMotor = new WPI_TalonSRX(5);

  private final Encoder encoder = new Encoder(0, 2);

  private final ArmFeedforward feedforward = new ArmFeedforward(2, 3, 15, 3);
  private final PIDController pid = new PIDController(0.04, 0, 0);



  public ArmSubsystem() {

    leftArmMotor.setInverted(true);
    rightArmMotor.setInverted(false);

    leftArmMotor.setNeutralMode(NeutralMode.Brake);
    rightArmMotor.setNeutralMode(NeutralMode.Brake);

    encoder.setDistancePerPulse(360.00/2048.00);
  }

  public void hold(double CurrentAngle){

    final double feed = feedforward.calculate(Math.toRadians(CurrentAngle), 0);

    // use PID, Testing with Profiled PID
    final double output =
        pid.calculate(getRatePosition(), 0);

    leftArmMotor.setVoltage(output + feed);
    rightArmMotor.setVoltage(output + feed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("arm Position", getArmPosition());
    SmartDashboard.putBoolean("arm more than 107", getArmPosition()>107);

  }

  public void setArmSpeed(double speed) {

    leftArmMotor.set(speed);
    rightArmMotor.set(speed);
  }

  public double getArmPosition(){
    return encoder.getDistance()+80.0;
  }

  public double getRatePosition(){
    return encoder.getRate();
  }
  


  // private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  // private final MutableMeasure<Angle> m_distance = mutable(Degrees.of(0));
  // // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  // private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(DegreesPerSecond.of(0));

  // // Create a new SysId routine for characterizing the drive.
  // private final SysIdRoutine m_sysIdRoutine =
  //     new SysIdRoutine(
  //         // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
  //         new SysIdRoutine.Config(),
  //         new SysIdRoutine.Mechanism(
  //             // Tell SysId how to plumb the driving voltage to the motors.
  //             (Measure<Voltage> volts) -> {
  //               leftArmMotor.setVoltage(volts.in(Volts));
  //               rightArmMotor.setVoltage(volts.in(Volts));
  //             },
  //             // Tell SysId how to record a frame of data for each motor on the mechanism being
  //             // characterized.
  //             log -> {
  //               // Record a frame for the left motors.  Since these share an encoder, we consider
  //               // the entire group to be one motor.
  //               log.motor("arm")
  //                   .voltage(
  //                       m_appliedVoltage.mut_replace(
  //                           leftArmMotor.get() * RobotController.getBatteryVoltage(), Volts))
  //                   .angularPosition(m_distance.mut_replace(getArmPosition(), Degrees))
  //                   .angularVelocity(
  //                       m_velocity.mut_replace(getRatePosition(), DegreesPerSecond));
  //               // Record a frame for the right motors.  Since these share an encoder, we consider
                
  //             },
  //             // Tell SysId to make generated commands require this subsystem, suffix test state in
  //             // WPILog with this subsystem's name ("drive")
  //             this));

  //   /**
  //  * Returns a command that will execute a quasistatic test in the given direction.
  //  *
  //  * @param direction The direction (forward or reverse) to run the test in
  //  */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutine.quasistatic(direction);
  // }

  // /**
  //  * Returns a command that will execute a dynamic test in the given direction.
  //  *
  //  * @param direction The direction (forward or reverse) to run the test in
  //  */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutine.dynamic(direction);
  // }

  
}


