// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Shooter.*;
import frc.robot.Constants;
import frc.robot.lib.InterpolatingDouble;
import frc.robot.lib.LinearServo;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_motorRight = new WPI_TalonFX(MOTOR_LEADER_CAN_ID, Constants.CANIVORE_CAN_BUS);
  private final WPI_TalonFX m_motorLeft = new WPI_TalonFX(MOTOR_FOLLOWER_CAN_ID, Constants.CANIVORE_CAN_BUS);
  private final DoubleSolenoid m_hood = new DoubleSolenoid(PneumaticsModuleType.REVPH, SOLENOID_FORWARD_ID, SOLENOID_REVERSE_ID);

  private double m_OffsetRPM = Constants.Shooter.Offset_RPM_Initial_Amount;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_motorRight.configFactoryDefault();
    // m_motorRight.configOpenloopRamp(1.5);
    m_motorRight.setNeutralMode(NeutralMode.Coast);
    m_motorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    // m_motorRight.configClosedloopRamp(0.5);
    m_motorRight.config_kF(0, 0.05); //get started umf (increases the actual base rpm exponentially or something) was.052 old BAT, new bat .0498, then was .055 for 3550 rpm, then .052
    m_motorRight.config_kP(0, 0.1); //p = push and oscillating once it gets there
    m_motorRight.config_kI(0, 0.0);
    m_motorRight.config_kD(0, 0.0);
    m_motorRight.configVoltageCompSaturation(12.0);
    m_motorRight.enableVoltageCompensation(true);
    m_motorLeft.configFactoryDefault();
    // m_motorLeft.configOpenloopRamp(1.5);
    m_motorLeft.setNeutralMode(NeutralMode.Coast);
    m_motorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    // m_motorLeft.configClosedloopRamp(0.5);
    m_motorLeft.setInverted(InvertType.InvertMotorOutput);
    m_motorLeft.configVoltageCompSaturation(12.0);
    m_motorLeft.enableVoltageCompensation(true);
    m_motorLeft.config_kF(0, 0.05); //get started umf (increases the actual base rpm exponentially or something) was.052 old BAT, new bat .0498, then was .055 for 3550 rpm, then .052
    m_motorLeft.config_kP(0, 0.1); //p = push and oscillating once it gets there
    m_motorLeft.config_kI(0, 0.0);
    m_motorLeft.config_kD(0, 0.0); //d =  dampening for the oscillation

    SmartDashboard.putNumber("Hood-Pos", 35);
    SmartDashboard.putNumber("RPM-Set", 2500 * SHOOTER_REDUCTION);
    SmartDashboard.putNumber("rpm-Offset", getOffsetRPM());

  }

  public void raiseHood() {
    m_hood.set(Value.kForward);
  }

  public void lowerHood() {
    m_hood.set(Value.kReverse);
  }

  public double getRpmSetpointError() {
    return (m_motorLeft.getClosedLoopError() / Constants.FALCON_ENCODER_RESOLUTION * 600.0);
  }

  public double getRPM(){
    return (m_motorRight.getSensorCollection().getIntegratedSensorVelocity() / Constants.FALCON_ENCODER_RESOLUTION * 600.0);

  }

  public void setRPM(final double rpm) {
    double velocity = (rpm * (Constants.FALCON_ENCODER_RESOLUTION) / 600.0); //1,000ms per sec, but robot cares about per 100ms, so then 60 sec/min
    m_motorRight.set(TalonFXControlMode.Velocity, velocity);
    m_motorLeft.set(TalonFXControlMode.Velocity, velocity);
  }
  
  public double getRPMFromLimelight() {
    final double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    return Constants.Targeting.kDistanceToShooter.getInterpolated(new InterpolatingDouble(ty)).value; //TODO Copied Code, replace with interpreted value of correct RPM speed 
    // return 1000; //Delete me when fixed
  }
  public double getHoodHeightFromLimelight() {
    final double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    return Constants.Targeting.kDistanceToShooter.getInterpolated(new InterpolatingDouble(ty)).value; //TODO Copied Code, replace with interpreted value of correct Hood height 
    // return 10; //Delete me when fixed
  }

  public void incrementOffsetRPM() {
    m_OffsetRPM = m_OffsetRPM + Constants.Shooter.Offset_RPM_Increment_Amount;
    SmartDashboard.putNumber("rpm-Offset", getOffsetRPM());
  }

  public void decrementOffsetRPM() {
    m_OffsetRPM = m_OffsetRPM - Constants.Shooter.Offset_RPM_Increment_Amount;
    SmartDashboard.putNumber("rpm-Offset", getOffsetRPM());
  }

  public double getOffsetRPM() {
    return m_OffsetRPM;
  }

  public void stop() {
    m_motorRight.stopMotor();
    m_motorLeft.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("hood position " + getHoodPosition());
    SmartDashboard.putNumber("RPM-Actual", getRPM());
    SmartDashboard.putNumber("rpm-error", getRpmSetpointError());
  }
}
