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

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX m_backtMotor;
  private final TalonFX m_frontMotor;
  
  public enum ClimberArm {
    kFront,
    kBack
  }

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_backtMotor = new TalonFX(Back_CLIMBER_MOTOR, Constants.CANIVORE_CAN_BUS);
    m_frontMotor = new TalonFX(Front_CLIMBER_MOTOR, Constants.CANIVORE_CAN_BUS);

    //Front motor should be inverted

    m_backtMotor.setInverted(InvertType.InvertMotorOutput);
    m_frontMotor.setInverted(InvertType.InvertMotorOutput);
    m_frontMotor.setSelectedSensorPosition(0);
    m_backtMotor.setSelectedSensorPosition(0);
    m_backtMotor.configVoltageCompSaturation(12.0);
    m_backtMotor.enableVoltageCompensation(true);
    m_frontMotor.configVoltageCompSaturation(12.0);
    m_frontMotor.enableVoltageCompensation(true);

    m_frontMotor.setNeutralMode(NeutralMode.Brake);
    m_backtMotor.setNeutralMode(NeutralMode.Brake);

  }

  public void zeroClimber() {
    m_frontMotor.setSelectedSensorPosition(0);
    m_backtMotor.setSelectedSensorPosition(0);
  }

  public void climberFrontExtend() {
    climberFrontExtend(CLIMBER_SPEED);
  }
  
  public void climberFrontExtend(final double overideSpeed) {
    m_frontMotor.set(TalonFXControlMode.PercentOutput, overideSpeed);
  }

  public void climberBackExtend() {
    climberBackExtend(CLIMBER_SPEED);
  }

  public void climberBackExtend(final double overideSpeed) {
    m_backtMotor.set(TalonFXControlMode.PercentOutput, overideSpeed);
  }

  public void climberFrontRetract() {
    climberFrontRetract(CLIMBER_SPEED);
  }

  public void climberFrontRetract(final double overideSpeed) {
    m_frontMotor.set(TalonFXControlMode.PercentOutput, -overideSpeed);
  }

  public void climberBackRetract() {
    climberBackRetract(CLIMBER_SPEED);
  }

  public void climberBackRetract(final double overideSpeed) {
    m_backtMotor.set(TalonFXControlMode.PercentOutput, -overideSpeed);
  }

  public void climberStop() {
    climberFrontStop();
    climberBackStop();
  }

  public void climberFrontStop() {
    m_frontMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
  public void climberBackStop() {
    m_backtMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
  private double getMotorPosition(TalonFX motor) {
    return Util.ticksToDistance(motor.getSelectedSensorPosition(), 2048, 1.0625*Math.PI, 15.34);
  }

  public double getBackPosition() {
    return getMotorPosition(m_backtMotor);
  }
  
  public double getFrontPosition() {
    return getMotorPosition(m_frontMotor);
  }

  public void coastBothMotors() {
    m_frontMotor.setNeutralMode(NeutralMode.Coast);
    m_backtMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void brakeBothMotors() {
    m_frontMotor.setNeutralMode(NeutralMode.Brake);
    m_backtMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Back Arm Pos", getBackPosition());
    SmartDashboard.putNumber("Front Arm Pos", getFrontPosition());
  }
}
