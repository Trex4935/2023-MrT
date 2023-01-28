// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoystickPortIDs;
import frc.robot.commands.ca_autoTrajectory;
import frc.robot.commands.ca_autoTurnKinematic;
import frc.robot.commands.ca_driveAutoSquare;
import frc.robot.commands.cm_driveWithJoysticks;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  // The robot's subsystems and commands are defined here...
  public final Drivetrain drivetrain;
  private final cm_driveWithJoysticks driveWithJoysticks;
  private final ca_autoTrajectory autoTrajectory;
  private final ca_autoTurnKinematic autoTurnTrajectory;
  private final ca_driveAutoSquare autoSquare;

  // triggers and buttons

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Joysticks
    leftJoystick = new Joystick(JoystickPortIDs.leftJoystickPortID);
    rightJoystick = new Joystick(JoystickPortIDs.rightJoystickPortID);
    // subsystems
    drivetrain = new Drivetrain();

    // commands
    driveWithJoysticks = new cm_driveWithJoysticks(drivetrain, leftJoystick, rightJoystick);
    drivetrain.setDefaultCommand(driveWithJoysticks);


    //Trajectory for autonomous
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      0.5, 1);

      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(0, 0.125),
        new Translation2d(0, 0.25)),
        //new Translation2d(xn, yn),
      new Pose2d(0, 0.5, Rotation2d.fromDegrees(0)),
      trajectoryConfig);

      autoTrajectory = new ca_autoTrajectory(drivetrain, trajectory);
      autoTurnTrajectory = new ca_autoTurnKinematic(drivetrain, 0.0, 90.0); // testing 90 degree Turn;
      autoSquare = new ca_driveAutoSquare(drivetrain,trajectory);

    SmartDashboard.putData(drivetrain);

    // Configure the button bindings
    configureButtonBindings();
    
    drivetrain.resetGyro();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoSquare;
  }
}
