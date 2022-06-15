// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.robot.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.*;

public class ClimberBackSubsystem extends SubsystemBase {
  /** Creates a new ClimberBackSubsystem. */
  private final TalonFX m_backMotor;

  public enum ClimberArm {
    kBack
  }

public ClimberBackSubsystem() {
  m_backMotor = new TalonFX(Back_CLIMBER_MOTOR, Constants.CANIVORE_CAN_BUS);

  m_backMotor.setInverted(InvertType.InvertMotorOutput);
  m_backMotor.setSelectedSensorPosition(0);
  m_backMotor.configVoltageCompSaturation(12.0);
  m_backMotor.enableVoltageCompensation(true);
  m_backMotor.setNeutralMode(NeutralMode.Brake);

}

public void zeroClimber() {
  m_backMotor.setSelectedSensorPosition(0);
}

public void climberExtend() {
  climberExtend(CLIMBER_SPEED);
}

public void climberExtend(final double overideSpeed) {
  m_backMotor.set(TalonFXControlMode.PercentOutput, overideSpeed);
}

public void climberRetract() {
  climberRetract(CLIMBER_SPEED);
}

public void climberRetract(final double overideSpeed) {
  m_backMotor.set(TalonFXControlMode.PercentOutput, -overideSpeed);
}

public void climberStop() {
  m_backMotor.set(TalonFXControlMode.PercentOutput, 0);
}

private double getMotorPosition(TalonFX motor) {
  return Util.ticksToDistance(motor.getSelectedSensorPosition(), 2048, 1.0625*Math.PI, 15.34);
}

public double getPosition() {
  return getMotorPosition(m_backMotor);
}

public void coastMotor() {
  m_backMotor.setNeutralMode(NeutralMode.Coast);
}

public void brakeMotor() {
  m_backMotor.setNeutralMode(NeutralMode.Brake);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Back Arm Pos", this.getPosition());
  }
}
