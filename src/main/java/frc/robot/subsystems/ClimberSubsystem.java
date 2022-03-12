// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.robot.Util;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.*;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX m_leftMotor;
  private final TalonFX m_rightMotor;
  private final DoubleSolenoid m_armPivot;
  private final DoubleSolenoid m_armLock;

  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_leftMotor = new TalonFX(LEFT_CLIMBER_MOTOR, Constants.CANIVORE_CAN_BUS);
    m_rightMotor = new TalonFX(RIGHT_CLIMBER_MOTOR, Constants.CANIVORE_CAN_BUS);
    m_armLock = new DoubleSolenoid(PneumaticsModuleType.REVPH, CLIMBER_PANCAKE_ONE, CLIMBER_PANCAKE_TWO);
    m_armPivot = new DoubleSolenoid(PneumaticsModuleType.REVPH, CLIMBER_PISTON_ONE, CLIMBER_PISTON_TWO);

    //Right motor should be inverted

    m_leftMotor.setInverted(InvertType.InvertMotorOutput);
    m_rightMotor.setInverted(InvertType.InvertMotorOutput);
    m_rightMotor.setSelectedSensorPosition(0);
    m_leftMotor.setSelectedSensorPosition(0);

    m_rightMotor.setNeutralMode(NeutralMode.Brake);
    m_leftMotor.setNeutralMode(NeutralMode.Brake);

  }

  public void lock() {
    m_armLock.set(Value.kForward); 
  }

  public void unlock() {
    m_armLock.set(Value.kReverse);
  }

  public void pivotDown() {
    m_armPivot.set(Value.kForward);
  }

  public void pivotUp() {
    m_armPivot.set(Value.kReverse);
  }

  public void climberRightExtend() {
    m_rightMotor.set(TalonFXControlMode.PercentOutput, 0.35);
  }

  public void climberLeftExtend() {
    m_leftMotor.set(TalonFXControlMode.PercentOutput, 0.35);
  }

  public void climberRightRetract() {
    m_rightMotor.set(TalonFXControlMode.PercentOutput, -0.35);
  }

    public void climberLeftRetract() {
      m_leftMotor.set(TalonFXControlMode.PercentOutput, -0.35);
    }

  public void climberStop() {
    climberRightStop();
    climberLeftStop();
  }

  public void climberRightStop() {
    m_rightMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
  public void climberLeftStop() {
    m_leftMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  private double getMotorPosition(TalonFX motor) {
    return Util.ticksToDistance(motor.getSelectedSensorPosition(), 2048, 1.0625*Math.PI, 15.34);
  }

  public double getLeftPosition() {
    return getMotorPosition(m_leftMotor);
  }

  public double getRightPosition() {
    return getMotorPosition(m_rightMotor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Arm Pos", getLeftPosition());
    SmartDashboard.putNumber("Right Arm Pos", getRightPosition());
  }
}
