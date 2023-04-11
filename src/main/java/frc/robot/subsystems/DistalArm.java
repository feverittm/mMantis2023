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

public class DistalArm extends SubsystemBase {

  /** Creates a new Distal Arm. */
  private CANSparkMax rightDistalMotor;
  private CANSparkMax leftDistalMotor;

  private RelativeEncoder rightMotorEncoder;
  private RelativeEncoder leftMotorEncoder;
  private AnalogPotentiometer distalArmPot;
  private AnalogInput distalArmInput;

  public DistalArm() {
    rightDistalMotor = new CANSparkMax(Constants.SUBSYSTEM.ARM.LEFT_LOCAL_NEO_CAN_ID, MotorType.kBrushless);
    leftDistalMotor = new CANSparkMax(Constants.SUBSYSTEM.ARM.RIGHT_LOCAL_NEO_CAN_ID, MotorType.kBrushless);

    rightMotorEncoder = rightDistalMotor.getEncoder();
    leftMotorEncoder = leftDistalMotor.getEncoder();

    distalArmInput = new AnalogInput(Constants.SUBSYSTEM.ARM.LOCAL_POTENTIOMETER_ANALOG_CHANNEL);
    distalArmInput.setAverageBits(2);
    distalArmPot = new AnalogPotentiometer(distalArmInput);

    rightDistalMotor.restoreFactoryDefaults();
    leftDistalMotor.restoreFactoryDefaults();

    leftDistalMotor.follow(rightDistalMotor);

    rightDistalMotor.setInverted(Constants.SUBSYSTEM.ARM.RIGHT_LOCAL_NEO_INVERTED);
    leftDistalMotor.setInverted(Constants.SUBSYSTEM.ARM.LEFT_LOCAL_NEO_INVERTED);
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
    leftDistalMotor.setIdleMode(mode);
    rightDistalMotor.setIdleMode(mode);
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
  public double getDistalArmInput() {
    return distalArmInput.getVoltage();
  }
  /**
   * Returns the value of the analog poteniometer
   * 
   * @return The angle from the poteniometer
   */
  public double getDistalArmAngle() {
    return distalArmPot.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distal Arm Sensor Voltage", getDistalArmInput());
    SmartDashboard.putNumber("Distal Arm Analog", getDistalArmAngle());
    SmartDashboard.putNumber("Distal Arm Motor Encoder", getRightMotorPosition());
  }
}