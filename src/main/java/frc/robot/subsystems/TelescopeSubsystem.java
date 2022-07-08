// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.commands.climber.BrakeTelescope;
import frc.robot.commands.climber.CoastTelescope;
import frc.robot.commands.climber.DisengageFrictionBrakeTelescope;
import frc.robot.commands.climber.EngageFrictionBrakeClimber;
import frc.robot.commands.climber.EngageFrictionBrakeTelescope;
import frc.robot.commands.climber.TelescopeAutoHome;
import frc.robot.commands.climber.TelescopeGoToClosedLoop;
import frc.robot.commands.climber.ZeroTelescope;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Climber.*;


public class TelescopeSubsystem extends SubsystemBase {

  private static final double k_spoolCircumference = 1.0625 * Math.PI;
  private static final double k_postEncoderGearing = 15.34;
  private static final StatorCurrentLimitConfiguration k_currentLimit = new StatorCurrentLimitConfiguration(true, 70, 70, 0.2);
  private static final double k_maxExtensionInches = 20.5;
  public static final int k_maxExtensionTicks = 217000;  //front 232,000 back 217000
  public static final int k_maxFrontExtensionTicks = 240000;
  public static final int k_maxBackExtensionTicks = 228000; 
  public static final int k_minExtensionTicks = 2500;



  private final TalonFX m_motor;
  private final String m_name;
  private int m_shuffleboardRowOffset = 0;
  private boolean m_isBrake = true;
  private boolean m_hasZeroed = false;
  private final Solenoid m_solenoid;
  private final int m_solenoidID;

  /** Creates a new TelescopeSubsystem. */
  public TelescopeSubsystem(final int motorId, boolean positiveIsForward, String name, int solenoidID) {
    m_name = name;
    m_solenoidID = solenoidID;
    ShuffleboardTab tab = Shuffleboard.getTab("Telescopes");

    m_motor = new TalonFX(motorId, Constants.CANIVORE_CAN_BUS);
    m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, m_solenoidID);
 
    // m_motor.setSelectedSensorPosition(0);
    // m_motor.configVoltageCompSaturation(4.0);
    // m_motor.enableVoltageCompensation(true);
    // m_motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 20, 0.2));
    // brakeMotor();

    m_motor.set(ControlMode.PercentOutput, 0.0);
    if (motorId == Constants.Climber.Back_CLIMBER_MOTOR) {
      m_shuffleboardRowOffset = 1;
    }
    if (isBottomedOut()) {
      zeroTelescope();
    } else {
      m_motor.setSelectedSensorPosition(0.0);
    }
    if (positiveIsForward == false) {
      m_motor.setInverted(InvertType.InvertMotorOutput);
    }
    m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    m_motor.configMotionAcceleration(40000, 10);
    m_motor.configMotionCruiseVelocity(20000, 10);
    m_motor.configAllowableClosedloopError(0, 2000, 10);
    m_motor.config_kP(0, 0.6);
    m_motor.config_kI(0, 0);
    m_motor.config_kD(0, 0);
    m_motor.config_kF(0, 0.79);
    brakeMotor();
    m_motor.configVoltageCompSaturation(12.0);
    m_motor.enableVoltageCompensation(true);
    m_motor.configStatorCurrentLimit(k_currentLimit);
    disengageBrake();

    // tab.addNumber(m_name + " Arm Pos",() -> getPositionInches()).withPosition(0, 2+m_shuffleboardRowOffset);
    tab.addNumber(m_name + " Arm Pos Ticks", () -> getPositionTicks()).withPosition(1, 2+m_shuffleboardRowOffset);
    // tab.addNumber(m_name + " Voltage", () -> m_motor.getBusVoltage()).withPosition(2, 2+m_shuffleboardRowOffset);
    // tab.addNumber(m_name + " Current", () -> m_motor.getStatorCurrent()).withPosition(3, 2+m_shuffleboardRowOffset);
    // tab.addBoolean(m_name + " Is Brake", () -> m_isBrake).withPosition(4, 2+m_shuffleboardRowOffset);
    tab.addBoolean(m_name + " Is Bottomed Out", () -> isBottomedOut()).withPosition(5, 2+m_shuffleboardRowOffset);
    // if (motorId == Constants.Climber.Front_CLIMBER_MOTOR) { // limit switch not working on back telescope right now
      tab.add(m_name + " Auto Home", new TelescopeAutoHome(this)).withPosition(6, 2+m_shuffleboardRowOffset);
      tab.add(m_name + " Go To 5", new TelescopeGoToClosedLoop(this, 46000)).withPosition(7, 2+m_shuffleboardRowOffset);
      tab.add(m_name + " Go To 10", new TelescopeGoToClosedLoop(this, 95000)).withPosition(8, 2+m_shuffleboardRowOffset);
      tab.add(m_name + " Go To 15", new TelescopeGoToClosedLoop(this, 143000)).withPosition(9, 2+m_shuffleboardRowOffset);
      tab.add(m_name + " Go To max", new TelescopeGoToClosedLoop(this, this.k_maxExtensionTicks)).withPosition(10, 2+m_shuffleboardRowOffset);
    // }

    tab.add(m_name + " Break Motor", new BrakeTelescope(this));
    tab.add(m_name + " Coast Motor", new CoastTelescope(this));
    tab.add(m_name + "  Zero", new ZeroTelescope(this));
  }

  public void zeroTelescope() {
    m_motor.setSelectedSensorPosition(0.0);
    m_hasZeroed = true;
  }
  
  /**
   * Extends with a default speed.
   */
  public void extend() {
    extend(CLIMBER_SPEED);
  }
  
  public void extend(final double overideSpeed) {
    m_motor.set(TalonFXControlMode.PercentOutput, overideSpeed);
  }

  public void setVoltageSaturation() {
    setVoltageSaturation(12.0);
  }

  public void setVoltageSaturation(double volts) {
    m_motor.configVoltageCompSaturation(volts);
  }
  
  /**
   * Retracts with a default speed.
   */
  public void retract() {
    retract(CLIMBER_SPEED);
  }
  
  public void retract(final double overideSpeed) {
    m_motor.set(TalonFXControlMode.PercentOutput, -overideSpeed);
  }
  
  public void stopTelescope() {
    m_motor.set(TalonFXControlMode.PercentOutput, 0);
  }
  
  public double getPositionInches() {
    return Util.ticksToDistance(
      getPositionTicks(),
      Constants.FALCON_ENCODER_RESOLUTION,
      k_spoolCircumference,
      k_postEncoderGearing
    );
  }
  
  public double getPositionTicks() {
    return m_motor.getSelectedSensorPosition();
  }
  
  public void setPositionTicks(double position) {
    // m_motor.set(TalonFXControlMode.Position, position);
    m_motor.set(TalonFXControlMode.MotionMagic, position);
  }

  public void setPositionInches(double position) {
    setPositionTicks(Util.distanceToTicks(position, Constants.FALCON_ENCODER_RESOLUTION, k_spoolCircumference, k_postEncoderGearing));
  }
  
  public void coastMotor() {
    m_motor.setNeutralMode(NeutralMode.Coast);
    m_isBrake = false;
  }
  
  public void brakeMotor() {
    m_motor.setNeutralMode(NeutralMode.Brake);
    m_isBrake = true;
  }

  public double getCurrent() {
    return m_motor.getStatorCurrent();
  }

  public boolean isBottomedOut() {
    final TalonFXSensorCollection sensorCollection = m_motor.getSensorCollection();
    return sensorCollection.isRevLimitSwitchClosed() == 1 || sensorCollection.isFwdLimitSwitchClosed() == 1;
  }

  public boolean hasZeroed() {
    return m_hasZeroed;
  }

  public void disengageBrake() {
    m_solenoid.set(true);
  }

  public void engageBrake() {
    m_solenoid.set(false);
  }

  @Override
  public void periodic() {
  }
}
