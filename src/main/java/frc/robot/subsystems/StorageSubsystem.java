// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Storage.*;

public class StorageSubsystem extends SubsystemBase {

  private int m_ballCount = INITIAL_BALL_COUNT;
  private final Debouncer m_debouncerBottom = new Debouncer(DEBOUNCE_DURATION_BOTTOM);
  private final Debouncer m_debouncerTop = new Debouncer(DEBOUNCE_DURATION_TOP);
  private final DigitalInput m_digitalInputBottom = new DigitalInput(DIGITAL_INPUT_BOTTOM_ID);
  private final DigitalInput m_digitalInputTop = new DigitalInput(DIGITAL_INPUT_TOP_ID);
  private boolean m_isBallAtBottom = false;
  private boolean m_isballAtTop = false;
  private final CANSparkMax m_motorBottom = new CANSparkMax(MOTOR_BOTTOM_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_motorTop = new CANSparkMax(MOTOR_TOP_CAN_ID, MotorType.kBrushless);

  /** Creates a new StorageSubsystem. */
  public StorageSubsystem() {
    m_motorBottom.restoreFactoryDefaults();
    m_motorBottom.setIdleMode(IdleMode.kBrake); 
    m_motorTop.restoreFactoryDefaults();
    m_motorTop.setIdleMode(IdleMode.kBrake);
    stop();
  }

  public void decrementBallCount() {
    m_ballCount--;
  }

  public int getBallCount() {
    return m_ballCount;
  }

  public void incrementBallCount() {
    m_ballCount++;
  }

  public boolean isBallAtEntrance() {
    return m_isBallAtBottom;
  }

  public boolean isBallAtExit() {
    return m_isballAtTop;
  }

  public void runForIntake() {
    m_motorBottom.set(RUN_IN_SPEED_BOTTOM);
    m_motorTop.set(RUN_IN_SPEED_TOP);
  }

  public void runForOutput() {
    m_motorBottom.set(RUN_OUT_SPEED_BOTTOM);
  }

  public void runForShooter() {
    m_motorBottom.set(RUN_IN_SPEED_BOTTOM/3.0);
    m_motorTop.set(1);
  }

  public void setBallCount(final int ballCount) {
    m_ballCount = ballCount;
  }

  public void zeroBallCount() {
    setBallCount(0);
  }

  public void stop() {
    m_motorBottom.stopMotor();
    m_motorTop.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_isBallAtBottom = m_debouncerBottom.calculate(!m_digitalInputBottom.get());
    m_isballAtTop = m_debouncerTop.calculate(!m_digitalInputTop.get());
    SmartDashboard.putBoolean("ball-bottom", m_isBallAtBottom);
    SmartDashboard.putBoolean("ball-top", m_isballAtTop);
    SmartDashboard.putNumber("ball count", m_ballCount);
  }
}
