// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Drivetrain;

public class cm_driveWithJoysticks extends CommandBase {

  private final Drivetrain drive;
  private final Joystick joystick1, joystick2;

  /** Creates a new c_driveWithControler. */
  public cm_driveWithJoysticks(Drivetrain dt, Joystick jst1, Joystick jst2) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = dt;
    joystick1 = jst1;
    joystick2 = jst2;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drive.driveWithJoysticks(joystick1, joystick2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopMotors();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}