// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Creates an Arm subsystem with 2 Neo motors, One will need to be inverted from the other.
 * 
 */

public class LocalArm extends SubsystemBase {

  /** Creates a new Local Arm. */
  private CANSparkMax rightLocalMotor;
  private CANSparkMax leftLocalMotor;

  private RelativeEncoder rightMotorEncoder;
  private RelativeEncoder leftMotorEncoder;
  private AnalogPotentiometer localArmPot;
  private AnalogInput localArmInput;

  public LocalArm() {
    rightLocalMotor = new CANSparkMax(Constants.SUBSYSTEM.ARM.LEFT_LOCAL_NEO_CAN_ID, MotorType.kBrushless);
    leftLocalMotor = new CANSparkMax(Constants.SUBSYSTEM.ARM.RIGHT_LOCAL_NEO_CAN_ID, MotorType.kBrushless);

    rightMotorEncoder = rightLocalMotor.getEncoder();
    leftMotorEncoder = leftLocalMotor.getEncoder();

    localArmInput = new AnalogInput(Constants.SUBSYSTEM.ARM.LOCAL_POTENTIOMETER_ANALOG_CHANNEL);
    localArmInput.setAverageBits(2);
    localArmPot = new AnalogPotentiometer(localArmInput);

    rightLocalMotor.restoreFactoryDefaults();
    leftLocalMotor.restoreFactoryDefaults();

    leftLocalMotor.follow(rightLocalMotor);

    rightLocalMotor.setInverted(Constants.SUBSYSTEM.ARM.RIGHT_LOCAL_NEO_INVERTED);
    leftLocalMotor.setInverted(Constants.SUBSYSTEM.ARM.LEFT_LOCAL_NEO_INVERTED);
    setMotorNeutralMode(IdleMode.kBrake);
  }

  /**
   * Sets the position of the two measured motor encoders to zero.
   */
  public void resetEncoders() {
    leftMotorEncoder.setPosition(0);
    rightMotorEncoder.setPosition(0);
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
    leftLocalMotor.setIdleMode(mode);
    rightLocalMotor.setIdleMode(mode);
  }

  /**
   * Returns the displacement of the right motor encoder.
   * 
   * @return The position of the measuring right motor encoder (encoder ticks).
   */
  public double getRightMotorPosition() {
    return rightMotorEncoder.getPosition();
  }

  /**
   * Returns the value of the analog poteniometer
   * 
   * @return The angle from the poteniometer
   */
  public double getLocalArmInput() {
    return localArmInput.getVoltage();
  }
  /**
   * Returns the value of the analog poteniometer
   * 
   * @return The angle from the poteniometer
   */
  public double getLocalArmAngle() {
    return localArmPot.get();
  }

  public void driveLocalArm(double speed) {
    rightLocalMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Local Arm Sensor Voltage", getLocalArmInput());
    SmartDashboard.putNumber("Local Arm Analog", getLocalArmAngle());
    SmartDashboard.putNumber("Local Arm Motor Encoder", getRightMotorPosition());
  }
}