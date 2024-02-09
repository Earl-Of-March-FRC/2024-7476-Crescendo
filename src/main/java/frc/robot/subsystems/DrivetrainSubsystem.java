// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */

  private final WPI_TalonFX leftFront = new WPI_TalonFX(1);
  private final WPI_TalonFX leftBack = new WPI_TalonFX(2);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(3);
  private final WPI_TalonFX rightBack = new WPI_TalonFX(4);

  private DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);

  ADXRS450_Gyro gyro = new ADXRS450_Gyro();


  public Pose3d visionPose;
  private double[] poseArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);

  private DifferentialDrivePoseEstimator poseEstimator;
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(1); // distance between left and right wheels in metres


  public DrivetrainSubsystem() {

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    leftFront.setInverted(true);
    leftBack.setInverted(true);

    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);

    leftFront.setSelectedSensorPosition(0);
    leftBack.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);

    gyro.reset();
    gyro.calibrate();

    // leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    visionPose = new Pose3d(poseArray[0], poseArray[1], poseArray[2], new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));
    poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), getLeftDistance(), getRightDistance(), visionPose.toPose2d());
  }

  public void tankDrive(DoubleSupplier left, DoubleSupplier right){
    drive.tankDrive(left.getAsDouble(), right.getAsDouble());
  }

  public double getLeftDistance(){
    return Units.inchesToMeters(leftFront.getSelectedSensorPosition() /   2048  / 12.75 *  (Math.PI * 2 * 2));
    //                                            encoder   gearratio    2PI * Wheel Radius in METRES
  }

  public double getRightDistance(){
    return Units.inchesToMeters(rightFront.getSelectedSensorPosition() /   2048  / 12.75 *  (Math.PI * 2 * 2));
    //                                            encoder   gearratio    2PI * Wheel Radius  in METRES
  }


  public Pose2d getBotPose(){
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    
    visionPose = new Pose3d(poseArray[0], poseArray[1], poseArray[2], new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));

    SmartDashboard.putNumber("Vision X", visionPose.getTranslation().getX());
    SmartDashboard.putNumber("Vision Y", visionPose.getTranslation().getY());
    SmartDashboard.putNumber("Vision Rotation", Math.toDegrees(visionPose.getRotation().getAngle()));

    poseEstimator.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());


    NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");

    if(leftFront.get() < 0.4 && rightFront.get() < 0.4 && ty != null && tx != null){
      poseEstimator.addVisionMeasurement(visionPose.toPose2d(), Timer.getFPGATimestamp());
    }

    SmartDashboard.putNumber("Bot X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Bot Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Bot Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

  }
}
