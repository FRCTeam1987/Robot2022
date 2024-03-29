// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Collector.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class CollectorSubsystem extends SubsystemBase {

  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, SOLENOID_FORWARD_ID, SOLENOID_REVERSE_ID);
  private final TalonFX m_motor = new TalonFX(MOTOR_CAN_ID, Constants.CANIVORE_CAN_BUS);

  /** Creates a new CollectorSubsystem. */
  public CollectorSubsystem() {
    m_motor.configFactoryDefault();
    m_motor.setNeutralMode(NeutralMode.Coast);
    stow();
    stop();
  }

  public void deploy() {
    m_doubleSolenoid.set(Value.kForward);
  }

  public void stow() {
    m_doubleSolenoid.set(Value.kReverse);
  }

  public void runRoller(final double speed) {
    m_motor.set(ControlMode.PercentOutput, speed);
  }

  public void runRollerIn() {
    m_motor.set(ControlMode.PercentOutput, RUN_IN_SPEED);

  }

  public void runRollerOut() {
    m_motor.set(ControlMode.PercentOutput, RUN_OUT_SPEED);

    
  }public void runAutoOut() {
    m_motor.set(ControlMode.PercentOutput, .25);

  }

  public void runRollerOutSlow() {
    m_motor.set(ControlMode.PercentOutput, RUN_OUT_SPEED/2.0);

  }

  public void stop() {
    m_motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
