// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DrivetrainConstants;
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
  private final AHRS gyro = new AHRS(Port.kOnboard);


  public Pose3d visionPose;

  public Field2d field = new Field2d();
  private double[] poseArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);


  private DifferentialDrivePoseEstimator poseEstimator;
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.8)); // distance between left and right wheels in metres

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainConstants.ks, DrivetrainConstants.kv);
  private PIDController leftPIPidController = new PIDController(DrivetrainConstants.P, 0, 0);
  private PIDController rightPIPidController = new PIDController(DrivetrainConstants.P, 0, 0);

  private ProfiledPIDController proPidLeft = new ProfiledPIDController(DrivetrainConstants.P, 0, 0, new TrapezoidProfile.Constraints(1, 0.5));
  private ProfiledPIDController proPidRight = new ProfiledPIDController(DrivetrainConstants.P, 0, 0, new TrapezoidProfile.Constraints(1, 0.5));

  public DrivetrainSubsystem() {    

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

    gyro.zeroYaw();
    gyro.reset();

    // visionPose = new Pose3d(poseArray[0], poseArray[1], poseArray[2], new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));

    Pose2d reset = new Pose2d(1.8, 7.37, new Rotation2d().fromDegrees(90));
    poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), getLeftDistance(), getRightDistance(), new Pose2d());


    AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // Current ChassisSpeeds supplier
            this::chassisSpeedsDrive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void tankDrive(DoubleSupplier left, DoubleSupplier right) {
    double leftDouble = left.getAsDouble();
    double rightDouble = right.getAsDouble();

    drive.setDeadband(0.15);

    drive.tankDrive(
      Math.signum(leftDouble)*(Math.sqrt(Math.abs(leftDouble))),
      Math.signum(rightDouble)*(Math.sqrt(Math.abs(rightDouble)))
    );
  }

  public void arcadeDrive(DoubleSupplier throttle, DoubleSupplier turn){
    drive.arcadeDrive(throttle.getAsDouble(), turn.getAsDouble());
  }

  public void chassisSpeedsDrive(ChassisSpeeds chassis){

    DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(chassis);

    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    // use PID, Testing with Profiled PID
    final double leftOutput =
        leftPIPidController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightPIPidController.calculate(getRightVelocity(), speeds.rightMetersPerSecond);

    leftFront.setVoltage(leftOutput + leftFeedforward);
    rightFront.setVoltage(rightOutput + rightFeedforward);
    drive.feed();
  }

    // Assuming this is a method in your drive subsystem
  public Command toSource() {
      PathPlannerPath path = PathPlannerPath.fromPathFile("ToSource");

      PathConstraints constraints = new PathConstraints(
        3.0, 2.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

      return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command toAmp() {
      PathPlannerPath path = PathPlannerPath.fromPathFile("ToAmp");


      PathConstraints constraints = new PathConstraints(
        3.0, 2.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

      return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public void resetPose(Pose2d pose){
    poseEstimator.resetPosition(gyro.getRotation2d(), new DifferentialDriveWheelPositions(getLeftDistance(), getRightDistance()), pose);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(getWheelSpeeds());
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


  public Pose2d getPose(){
    poseEstimator.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    return poseEstimator.getEstimatedPosition();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightDistance());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftFront.setVoltage(leftVolts);
    rightFront.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetGyro(){
    gyro.reset();
  }

  // private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  // private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  // private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // // Create a new SysId routine for characterizing the drive.
  // private final SysIdRoutine m_sysIdRoutine =
  //     new SysIdRoutine(
  //         // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
  //         new SysIdRoutine.Config(),
  //         new SysIdRoutine.Mechanism(
  //             // Tell SysId how to plumb the driving voltage to the motors.
  //             (Measure<Voltage> volts) -> {
  //               leftFront.setVoltage(volts.in(Volts));
  //               rightFront.setVoltage(volts.in(Volts));
  //             },
  //             // Tell SysId how to record a frame of data for each motor on the mechanism being
  //             // characterized.
  //             log -> {
  //               // Record a frame for the left motors.  Since these share an encoder, we consider
  //               // the entire group to be one motor.
  //               log.motor("drive-left")
  //                   .voltage(
  //                       m_appliedVoltage.mut_replace(
  //                           leftFront.get() * RobotController.getBatteryVoltage(), Volts))
  //                   .linearPosition(m_distance.mut_replace(getLeftDistance(), Meters))
  //                   .linearVelocity(
  //                       m_velocity.mut_replace(getLeftVelocity(), MetersPerSecond));
  //               // Record a frame for the right motors.  Since these share an encoder, we consider
  //               // the entire group to be one motor.
  //               log.motor("drive-right")
  //                   .voltage(
  //                       m_appliedVoltage.mut_replace(
  //                           rightFront.get() * RobotController.getBatteryVoltage(), Volts))
  //                   .linearPosition(m_distance.mut_replace(getRightDistance(), Meters))
  //                   .linearVelocity(
  //                       m_velocity.mut_replace(getRightVelocity(), MetersPerSecond));
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


  @Override
  public void periodic() {
    
    field.setRobotPose(getBotPose());
    SmartDashboard.putData("Field", field);
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
    SmartDashboard.putNumber("Rotation", gyro.getRotation2d().getDegrees());



    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Right Distance", getRightDistance());

  }
}



// System Identification Tests

