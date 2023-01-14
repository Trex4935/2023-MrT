package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.InvertDrivetrain;
import frc.robot.Constants.MotorSpeedMultipliers;

public class Drivetrain extends SubsystemBase {

  WPI_TalonSRX leftTop;
  WPI_TalonSRX leftBottom;
  WPI_TalonSRX rightTop;
  WPI_TalonSRX rightBottom;

  MotorControllerGroup leftMotorGroup;
  MotorControllerGroup rightMotorGroup;

  int leftInvert;
  int rightInvert;

  double motorSpeedMultiplier;

  DifferentialDrive drive;

  /** Creates a new DriveTrain. */
  public Drivetrain() {
    // motor canbus
    leftTop = new WPI_TalonSRX(DrivetrainConstants.leftTopCanID);
    leftBottom = new WPI_TalonSRX(DrivetrainConstants.leftBottomCanID);
    rightTop = new WPI_TalonSRX(DrivetrainConstants.rightTopCanID);
    rightBottom = new WPI_TalonSRX(DrivetrainConstants.rightBottomCanID);

    // Default

    leftTop.configFactoryDefault();
    leftBottom.configFactoryDefault();
    rightBottom.configFactoryDefault();
    rightTop.configFactoryDefault();

    // invert
    leftInvert = InvertDrivetrain.leftInvert;
    rightInvert = InvertDrivetrain.rightInvert;

    // motor group
    leftMotorGroup = new MotorControllerGroup(leftTop, leftBottom);
    rightMotorGroup = new MotorControllerGroup(rightTop, rightBottom);
    rightMotorGroup.setInverted(true);

    // motor speed multiplier
    motorSpeedMultiplier = MotorSpeedMultipliers.motorSpeedMultiplier;



    // arcade drive
    drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    drive.setMaxOutput(Constants.highGear);

  }


  public void driveWithJoysticks(Joystick joystick1, Joystick joystick2) {
    drive.tankDrive(
        (((joystick1.getRawAxis(Constants.JoystickAxis)) * leftInvert)
            * MotorSpeedMultipliers.motorSpeedMultiplierLeft),
        (((joystick2.getRawAxis(Constants.JoystickAxis)) * rightInvert)
            * MotorSpeedMultipliers.motorSpeedMultiplierRight));
  }

  public void driveStraight(Double speed) {
    drive.tankDrive(speed, speed);
  }


  public void stopMotors() {
    leftTop.stopMotor();
    leftBottom.stopMotor();
    rightTop.stopMotor();
    rightBottom.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}