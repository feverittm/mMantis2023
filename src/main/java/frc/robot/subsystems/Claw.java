// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  private Solenoid clawSolenoid;

  /** Creates a new Claw. */
  public Claw() {
    clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.SUBSYSTEM.GRABBER.SOLENOID_CHANNEL);
  }

  public void openClaw() {
    clawSolenoid.set(true);
  }

  public void closeClaw() {
    clawSolenoid.set(false);
  }

  public void toggleClaw() {
    clawSolenoid.toggle();
  }
}
