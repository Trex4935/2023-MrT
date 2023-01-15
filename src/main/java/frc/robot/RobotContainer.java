// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoystickPortIDs;
import frc.robot.commands.cm_driveWithJoysticks;
import frc.robot.subsystems.Drivetrain;
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
  private final Drivetrain drivetrain;
  private final cm_driveWithJoysticks driveWithJoysticks;

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

    SmartDashboard.putData(drivetrain);

    // Configure the button bindings
    configureButtonBindings();
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
    return null;
  }
}
