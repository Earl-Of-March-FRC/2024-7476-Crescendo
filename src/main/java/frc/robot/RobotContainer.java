// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.CancelDriveAutos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDriveCmd;
import frc.robot.commands.test;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DrivetrainSubsystem drive = new DrivetrainSubsystem();
  private final ArmSubsystem armSub = new ArmSubsystem();
  private final ShooterSubsystem shooterSub = new ShooterSubsystem();
  private final IntakeSubsystem intakeSub = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController dController = new XboxController(0);
  private final SendableChooser<Command> autoChooser;

  private Command toSource = drive.toSource();
  private Command toAmp = drive.toAmp();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive.resetGyro();

    drive.setDefaultCommand(new TankDriveCmd(drive, () -> -dController.getLeftY(), () -> -dController.getRightY()));
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // new JoystickButton(dController, 2).whileTrue(new ArmCommand(armSub, 0.3));
    // new JoystickButton(dController, 3).whileTrue(new ArmCommand(armSub, -0.3));
    // new JoystickButton(dController, 4).whileTrue(new ShooterCommand(shooterSub, -0.3));
    // new JoystickButton(dController, 1).whileTrue(new IntakeCommand(intakeSub, 0.3));
    



    new JoystickButton(dController, 5).whileTrue(armSub.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    new JoystickButton(dController, 6).whileTrue(armSub.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    new JoystickButton(dController, 4).whileTrue(armSub.sysIdDynamic(SysIdRoutine.Direction.kForward));
    new JoystickButton(dController, 2).whileTrue(armSub.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  
    // begin path-planning branch contents
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // new JoystickButton(dController, 5).whileTrue(toSource);
    // new JoystickButton(dController,6).whileTrue(toAmp);
    // new JoystickButton(dController, 3).onTrue(new CancelDriveAutos(toSource, toAmp));
    // new JoystickButton(dController, 2).whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // end path-planning branch

    // new JoystickButton(dController, 3).onTrue(new test(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    return autoChooser.getSelected();

    // var autoVoltageConstraint =
    //         new DifferentialDriveVoltageConstraint(
    //             new SimpleMotorFeedforward(
    //                 DrivetrainConstants.ks,
    //                 DrivetrainConstants.kv,
    //                 DrivetrainConstants.ka),
    //             DrivetrainConstants.Dkinematics,
    //             7);

    //     // Create config for trajectory
    //     TrajectoryConfig config =
    //         new TrajectoryConfig(
    //                 DrivetrainConstants.kMaxSpeedMetersPerSecond,
    //                 DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
    //             // Add kinematics to ensure max speed is actually obeyed
    //             .setKinematics(DrivetrainConstants.Dkinematics)
    //             // Apply the voltage constraint
    //             .addConstraint(autoVoltageConstraint);

    //     // An example trajectory to follow. All units in meters.
    //     Trajectory exampleTrajectory =
    //         TrajectoryGenerator.generateTrajectory(
    //             // Start at the origin facing the +X direction
    //             new Pose2d(0, 0, new Rotation2d(0)),
    //             // Pass through these two interior waypoints, making an 's' curve path
    //             List.of(new Translation2d(2, 0.5), new Translation2d(2, -0.5), new Translation2d(0, 0)),
    //             // End 3 meters straight ahead of where we started, facing forward
    //             new Pose2d(4, 0, new Rotation2d(0)),
    //             // Pass config
    //             config);

    //     RamseteCommand ramseteCommand =
    //         new RamseteCommand(
    //             exampleTrajectory,
    //             drive::getPose,
    //             new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta),
    //             new SimpleMotorFeedforward(
    //                 DrivetrainConstants.ks,
    //                 DrivetrainConstants.kv,
    //                 DrivetrainConstants.ka),
    //             DrivetrainConstants.Dkinematics,
    //             drive::getWheelSpeeds,
    //             new PIDController(DrivetrainConstants.P, 0.0, 0.0),
    //             new PIDController(DrivetrainConstants.P, 0.0, 0.0),
    //             // RamseteCommand passes volts to the callback
    //             drive::tankDriveVolts,
    //             drive);


    // return ramseteCommand.andThen(Commands.runOnce(() -> drive.tankDriveVolts(0, 0)));
  }
}
