// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj.I2C.Port;



import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */

  private final TalonFX leftFront = new TalonFX(3);
  private final TalonFX leftBack = new TalonFX(4);
  private final TalonFX rightFront = new TalonFX(1);
  private final TalonFX rightBack = new TalonFX(2);

  private DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);



  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                leftFront.setVoltage(volts.in(Volts));
                rightFront.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            leftFront.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getLeftDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getLeftVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightFront.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getRightDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getRightVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));


  private final AHRS gyro = new AHRS(Port.kOnboard);


  public Pose3d visionPose;
  private double[] poseArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);

  private DifferentialDrivePoseEstimator poseEstimator;
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.8)); // distance between left and right wheels in metres


  public DrivetrainSubsystem() {    
    // leftFront.setInverted(tru e);
    // leftBack.setInverted(true);

    leftBack.setControl(new Follower(leftFront.getDeviceID(), false));
    rightBack.setControl(new Follower(rightFront.getDeviceID(), false));

    MotorOutputConfigs brake = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    MotorOutputConfigs invBrake = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);

    leftFront.getConfigurator().apply(brake);
    leftBack.getConfigurator().apply(brake);
    rightFront.getConfigurator().apply(invBrake);
    rightBack.getConfigurator().apply(invBrake);


    leftFront.setPosition(0);
    rightFront.setPosition(0);

    gyro.reset();

    // visionPose = new Pose3d(poseArray[0], poseArray[1], poseArray[2], new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));

    Pose2d reset = new Pose2d(0, 0, new Rotation2d(0));
    poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), getLeftDistance(), getRightDistance(), reset);
  }

  public void tankDrive(DoubleSupplier left, DoubleSupplier right){
    drive.tankDrive(left.getAsDouble(), right.getAsDouble());
  }

  public double getLeftDistance(){
    return Units.inchesToMeters(leftFront.getPosition().getValueAsDouble()  / 8.46 *  (Math.PI * 6));
    //                                               gearratio    2PI * Wheel Radius in METRES
  }

  public double getRightDistance(){
    return Units.inchesToMeters(rightFront.getPosition().getValueAsDouble()/ 8.46 *  (Math.PI * 6));
    //                                               gearratio    2PI * Wheel Radius  in METRES
  }

  public double getLeftVelocity(){
    return leftFront.getVelocity().getValueAsDouble()/ 8.46 *  Units.inchesToMeters((Math.PI * 6));
  }

  public double getRightVelocity(){
    return rightFront.getVelocity().getValueAsDouble()/ 8.46 *  Units.inchesToMeters((Math.PI * 6));

  }


  public Pose2d getBotPose(){
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }





  @Override
  public void periodic() {
    
    // visionPose = new Pose3d(poseArray[0], poseArray[1], poseArray[2], new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));

    // SmartDashboard.putNumber("Vision X", visionPose.getTranslation().getX());
    // SmartDashboard.putNumber("Vision Y", visionPose.getTranslation().getY());
    // SmartDashboard.putNumber("Vision Rotation", Math.toDegrees(visionPose.getRotation().getAngle()));

    poseEstimator.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());


    // NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    // NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");

    // if(leftFront.get() < 0.4 && rightFront.get() < 0.4 && ty != null && tx != null){
    //   poseEstimator.addVisionMeasurement(visionPose.toPose2d(), Timer.getFPGATimestamp());
    // }

    SmartDashboard.putNumber("Bot X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Bot Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Bot Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());


    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Right Distance", getRightDistance());

  }
}
