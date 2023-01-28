package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.InvertDrivetrain;
import frc.robot.Constants.MotorSpeedMultipliers;
import edu.wpi.first.wpilibj.SPI;

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
  double m_MaxSpeed;

  //Kinemtatics
  DifferentialDriveKinematics kin;

  //Simulate
  public double zSimAngle;

  //Gyro
  public static AHRS ahrs;


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
    // motorSpeedMultiplier = MotorSpeedMultipliers.motorSpeedMultiplier;



    // arcade drive
    drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    drive.setMaxOutput(Constants.highGear);

    
        // Distance between 2 wheel godzilla 641 mm, to do find or measure same for mrT
        kin = new DifferentialDriveKinematics(DrivetrainConstants.trackWidth);

    // initiate simulate gyro Position
    zSimAngle = 0;

    
        // Creating gyro object
        ahrs = new AHRS(SPI.Port.kMXP);

  }


  public void driveWithJoysticks(Joystick joystick1, Joystick joystick2) {
    drive.tankDrive(
        (((joystick1.getRawAxis(Constants.JoystickAxis)) * leftInvert)),
        (((joystick2.getRawAxis(Constants.JoystickAxis)) * rightInvert)));
  }

  public void driveStraight(Double speed) {
    drive.tankDrive(speed, speed);
  }

  public void driveTank(Double leftSpeed, Double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }


  public void stopMotors() {
    leftTop.stopMotor();
    leftBottom.stopMotor();
    rightTop.stopMotor();
    rightBottom.stopMotor();
  }

  private double getMaxSpeed(){
    return m_MaxSpeed;
  }

  private void setMaxSpeed(double MaxSpeed){
    m_MaxSpeed = MaxSpeed;
    drive.setMaxOutput(m_MaxSpeed);
  }

  // takes in chasis speed and chasis angular rate or rotation and return the left speed of the wheel;
  public double getLeftSpeedKin( double chassisSpeedx, double chassisAngularRate){
      double chassisSpeedy = 0;
      DifferentialDriveWheelSpeeds wheelSpeed = kin.toWheelSpeeds(new ChassisSpeeds(chassisSpeedx,chassisSpeedy, chassisAngularRate));
      return wheelSpeed.leftMetersPerSecond;
  }

  // takes in chasis speed and chasis angular rate or rotation and return the right speed of the wheel;
  public double getRightpeedKin(double chassisSpeedx, double chassisAngularRate){
      double chassisSpeedy = 0;
      DifferentialDriveWheelSpeeds wheelSpeed = kin.toWheelSpeeds(new ChassisSpeeds(chassisSpeedx,chassisSpeedy, chassisAngularRate));;
      return wheelSpeed.rightMetersPerSecond;
  }

  public void simulateGyro(double leftSpeed, double rightSpeed, Timer timer){

      ChassisSpeeds chassisSpeed = kin.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
      zSimAngle = chassisSpeed.omegaRadiansPerSecond * 0.02 + zSimAngle;
  }


    /** Resets the gyro */
    public void resetGyro() {
      ahrs.reset();
  }

  /** Gets Roll(X) angle from Gyro */
  public Float getXAngle() {
      return ahrs.getRoll();
  }

  /**  Gets Pitch(Y) angle from Gyro */
  public Float getYAngle() {
      return ahrs.getPitch();
  }

  /** Gets Yaw(Z) angle from Gyro */
  public Float getZAngle() {
      return ahrs.getYaw();
  }


  @Override
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("MaxSpeed", this::getMaxSpeed, this::setMaxSpeed);
    builder.addDoubleProperty("LeftTop", ()->leftTop.getMotorOutputVoltage(), null);
    builder.addDoubleProperty("leftBottom", ()->leftBottom.getMotorOutputVoltage(), null);
    builder.addDoubleProperty("rightTop", ()->rightTop.getMotorOutputVoltage(), null);
    builder.addDoubleProperty("rightBottom", ()->rightBottom.getMotorOutputVoltage(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}