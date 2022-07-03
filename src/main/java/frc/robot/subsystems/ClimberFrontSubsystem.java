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

public class ClimberFrontSubsystem extends SubsystemBase {
  /** Creates a new ClimberFrontSubsystem. */
  private final TalonFX m_frontMotor;

  public enum ClimberArm {
    kFront
  }
  
public ClimberFrontSubsystem() {
  m_frontMotor = new TalonFX(Front_CLIMBER_MOTOR, Constants.CANIVORE_CAN_BUS);

  m_frontMotor.setInverted(InvertType.InvertMotorOutput);
  m_frontMotor.setSelectedSensorPosition(0);
  m_frontMotor.configVoltageCompSaturation(12.0);
  m_frontMotor.enableVoltageCompensation(true);
  m_frontMotor.setNeutralMode(NeutralMode.Brake);

}

public void zeroClimber() {
  m_frontMotor.setSelectedSensorPosition(0);
}

public void climberExtend() {
  climberExtend(CLIMBER_SPEED);
}

public void climberExtend(final double overideSpeed) {
  m_frontMotor.set(TalonFXControlMode.PercentOutput, overideSpeed);
}

public void climberRetract() {
  climberRetract(CLIMBER_SPEED);
}

public void climberRetract(final double overideSpeed) {
  m_frontMotor.set(TalonFXControlMode.PercentOutput, -overideSpeed);
}

public void climberStop() {
  m_frontMotor.set(TalonFXControlMode.PercentOutput, 0);
}

private double getMotorPosition(TalonFX motor) {
  return Util.ticksToDistance(motor.getSelectedSensorPosition(), 2048, 1.0625*Math.PI, 15.34);
}

public double getPosition() {
  return getMotorPosition(m_frontMotor);
}

public void coastMotor() {
  m_frontMotor.setNeutralMode(NeutralMode.Coast);
}

public void brakeMotor() {
  m_frontMotor.setNeutralMode(NeutralMode.Brake);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Front Arm Pos", this.getPosition());
  }
}
