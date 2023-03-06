// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private CANSparkMax frontRight;
  private CANSparkMax frontLeft;
  private CANSparkMax backRight;
  private CANSparkMax backLeft;
  private final DifferentialDrive m_drive;
  private AHRS gyro;
  private Solenoid shiftSolenoid;
  private RelativeEncoder RightMotorEncoder;
  private RelativeEncoder LeftMotorEncoder;
  private static boolean shift_state = false;


  /**
   * Creates a Drivetrain subsystem with 4 Neo motors arranged 2 on
   * a side.
   * 
   */
  public Drivetrain() {

    frontRight = new CANSparkMax(Constants.SUBSYSTEM.DRIVETRAIN.FRONT_RIGHT_CAN_ID, MotorType.kBrushless);
    frontLeft = new CANSparkMax(Constants.SUBSYSTEM.DRIVETRAIN.FRONT_LEFT_CAN_ID, MotorType.kBrushless);
    backRight = new CANSparkMax(Constants.SUBSYSTEM.DRIVETRAIN.BACK_RIGHT_CAN_ID, MotorType.kBrushless);
    backLeft = new CANSparkMax(Constants.SUBSYSTEM.DRIVETRAIN.BACK_LEFT_CAN_ID, MotorType.kBrushless);


    shiftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    RightMotorEncoder = frontRight.getEncoder();
    LeftMotorEncoder = frontLeft.getEncoder();

    frontRight.restoreFactoryDefaults();
    frontLeft.restoreFactoryDefaults();
    backRight.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();

    backRight.follow(frontRight);
    backLeft.follow(frontLeft);

    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    frontRight.setInverted(false);
    backRight.setInverted(false);

    m_drive = new DifferentialDrive(backRight, backLeft);

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus. */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      gyro = new AHRS();
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    resetEncoders();
    resetGyroAngle();
  }

  /**
   * Sets the position of the two measured motor encoders to zero.
   */
  public void resetEncoders() {
    LeftMotorEncoder.setPosition(0);
    RightMotorEncoder.setPosition(0);
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Sets the left and right values for the drivetrain motors.
   * 
   * @param right : Value between [-1,1] (inclusive) to set the right-side motors
   *              to.
   * @param left  : Value between [-1,1] (inclusive) to set the left-side motors
   *              to.
   */
  public void basicMove(double right, double left) {
    frontLeft.set(left);
    frontRight.set(right);
  }

  /**
   * Sets the motors of the drivetrain to a specified neutral mode to go to when
   * an input of zero or null is recieved.
   * <p>
   * NeutralMode.Brake : The motors will attempt to go to zero velocity.
   * <p>
   * NeutralMode.Coast : The motors will not apply current.
   * 
   * @param mode : The mode to set to.
   */
  public void setMotorNeutralMode(IdleMode mode) {
    frontLeft.setIdleMode(mode);
    frontRight.setIdleMode(mode);
    backLeft.setIdleMode(mode);
    backRight.setIdleMode(mode);
  }

  /**
   * Returns the angle of the onboard gyroscope.
   * 
   * @return The angle output by the gyroscope (degrees, clockwise from positive
   *         Y axis).
   */
  public double getGyroAngle() {
      return gyro.getAngle();
  }

  /**
   * Sets the angle of the gyroscope to zero. To maintain a proper field-relative
   * angle, do not call this during a match.
   */
  public void resetGyroAngle() {
      gyro.reset();
  }

  /**
   * Returns the displacement of the right encoder.
   * 
   * @return The position of the measuring right motor encoder (encoder ticks).
   */
  public double getRightSensorPosition() {
    return RightMotorEncoder.getPosition();
  }

  /**
   * Returns the displacement of the left encoder.
   * 
   * @return The position of the measuring left motor encoder (encoder ticks).
   */
  public double getLeftSensorPosition() {
    return LeftMotorEncoder.getPosition();
  }

  /**
   * Returns the rate of the right encoder.
   * 
   * @return The velocity of the measuring right motor encoder (encoder ticks
   *         per second).
   */
  public double getRightSensorVelocity() {
    return 10 * RightMotorEncoder.getVelocity();
  }

  /**
   * Returns the rate of the left encoder.
   * 
   * @return The velocity of the measuring left motor encoder (encoder ticks per
   *         second).
   */
  public double getLeftSensorVelocity() {
    return -10 * RightMotorEncoder.getVelocity();
  }

  public void toggleShifter() {
    if (shift_state) {
      shift_state = false;
      shiftSolenoid.set(false);
    } else {
      shift_state = true;
      shiftSolenoid.set(true);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left actual v",
        getLeftSensorVelocity());
    SmartDashboard.putNumber("right actual v",
        getRightSensorVelocity());
  }
}