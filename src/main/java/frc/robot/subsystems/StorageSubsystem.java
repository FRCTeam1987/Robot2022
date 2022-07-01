// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.Constants.Storage;
import frc.robot.commands.shooter.EjectOneBallBottom;
import frc.robot.commands.storage.SetBallCount;

import static frc.robot.Constants.Storage.*;

import java.util.function.BooleanSupplier;

public class StorageSubsystem extends SubsystemBase {

  private int m_ballCount = INITIAL_BALL_COUNT;
  private final Debouncer m_debouncerBottom = new Debouncer(DEBOUNCE_DURATION_BOTTOM_LB);
  private final Debouncer m_debouncerTop = new Debouncer(DEBOUNCE_DURATION_TOP_LB);
  private final Debouncer m_debouncerColorBottom = new Debouncer(DEBOUNCE_DURATION_BOTTOM_C);
  private final Debouncer m_debouncerColorTop = new Debouncer(DEBOUNCE_DURATION_TOP_C);
  private final DigitalInput m_digitalInputBottom = new DigitalInput(DIGITAL_INPUT_BOTTOM_ID);
  private final DigitalInput m_digitalInputTop = new DigitalInput(DIGITAL_INPUT_TOP_ID);
  private boolean m_isBallAtBottom = false;
  private boolean m_isballAtTop = false;
  private final CANSparkMax m_motorBottom = new CANSparkMax(MOTOR_BOTTOM_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_motorTop = new CANSparkMax(MOTOR_TOP_CAN_ID, MotorType.kBrushless);
  private boolean m_hasFirstBall = false;
  private boolean m_hasSecondBall = false;
  private boolean m_UseColorSensorForBallCount = false;
  
  // private final NetworkTable table;
  // private NetworkTableEntry colorTop;
  // private NetworkTableEntry colorBottom;
  // private NetworkTableEntry prox;


  /** Creates a new StorageSubsystem. */
  public StorageSubsystem() {//NetworkTable table) {
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab.addBoolean("ball-bottom",() -> m_isBallAtBottom);
    shooterTab.addBoolean("ball-top", () -> m_isballAtTop);
    shooterTab.addNumber("ball count", () -> m_ballCount);
    shooterTab.add("0 Count", new SetBallCount(this, 0));
    shooterTab.add("1 Count", new SetBallCount(this, 1));
    shooterTab.add("2 Count", new SetBallCount(this, 2));

    m_motorBottom.restoreFactoryDefaults();
    m_motorBottom.setIdleMode(IdleMode.kBrake); 
    m_motorTop.restoreFactoryDefaults();
    m_motorTop.setIdleMode(IdleMode.kBrake);
    stop();
    // SmartDashboard.putBoolean("Should Use Color Sensor", m_UseColorSensorForBallCount);
    
   
    // this.table = table;
    // colorTop = table.getEntry("rgbir");   
    // colorBottom = table.getEntry("rgbir");   
    // prox = table.getEntry("proximity");


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

  public void runForOutputSlow() {
    m_motorBottom.set(RUN_OUT_SPEED_BOTTOM/2.0);
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

  public boolean hasFirstBall() {
    return m_hasFirstBall;
  }

  public boolean hasSecondBall() {
    return m_hasSecondBall;
  }

  public void stop() {
    m_motorBottom.stopMotor();
    m_motorTop.stopMotor();
  }

  public double[] getColorTop() {
    return NetworkTableInstance.getDefault().getEntry("/rawcolor3").getDoubleArray(DEFAULT_COLOR);
  }

  public boolean hasColorTop() {
    return getColorTopProximity() > MAX_COLOR_SENSOR_PROXIMITY;
  }

  public double getColorTopProximity() {
    return NetworkTableInstance.getDefault().getEntry("/proximity3").getDouble(0);
  }

  public double[] getColorBottom() {
    return NetworkTableInstance.getDefault().getEntry("/rawcolor1").getDoubleArray(DEFAULT_COLOR);
  }

  public double getColorBottomProximity() {
    return NetworkTableInstance.getDefault().getEntry("/proximity1").getDouble(0);
  }

  public boolean hasColorBottom() {
    return getColorBottomProximity() > MAX_COLOR_SENSOR_PROXIMITY;
  }

  public boolean isTopRed(){
    return isBallRed(getColorTop(), getColorTopProximity());
  }

  public boolean isTopBlue(){
    return isBallBlue(getColorTop(), getColorTopProximity());
  }

  public boolean isBottomRed(){
    return isBallRed(getColorBottom(), getColorBottomProximity());
  }

  public boolean isBottomBlue(){
    return isBallBlue(getColorBottom(), getColorBottomProximity());
  }

  public boolean isBottomOurs() {
    return this.hasColorBottom() && (
      (DriverStation.getAlliance().compareTo(Alliance.Blue) == 0 && this.isBottomBlue())
      || (DriverStation.getAlliance().compareTo(Alliance.Red) == 0 && this.isBottomRed())
    );
  }
  public boolean isTopOurs() {
    return this.hasColorTop() && (
      (DriverStation.getAlliance().compareTo(Alliance.Blue) == 0 && this.isTopBlue())
      || (DriverStation.getAlliance().compareTo(Alliance.Red) == 0 && this.isTopRed())
    );
  }

  private boolean colorIsWithinTolerance(double[] measuredColor, double[] knownColor) {
    return Util.isWithinTolerance(measuredColor[0], knownColor[0], COLOR_TOLERANCE[0])
      && Util.isWithinTolerance(measuredColor[1], knownColor[1], COLOR_TOLERANCE[1]) 
      && Util.isWithinTolerance(measuredColor[2], knownColor[2], COLOR_TOLERANCE[2])
      && Util.isWithinTolerance(measuredColor[3], knownColor[3], COLOR_TOLERANCE[3]);
  }

  private boolean isBallRed(double[] measuredColor, double proximity) {
    // return measuredColor[0] > 2250 && measuredColor[2] < 1500;
    return measuredColor[0] > measuredColor[2] && proximity > 200;
    // 5550, 3390, 1207, 85
  }

  private boolean isBallBlue(double[] measuredColor, double proximity) {
    // return measuredColor[2] > 1500 && measuredColor[0] < 2250;
    return measuredColor[0] < measuredColor[2] && proximity > 200;
    // 2630, 7100, 8280, 163
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_isBallAtBottom = m_debouncerBottom.calculate(!m_digitalInputBottom.get());
    m_isballAtTop = m_debouncerTop.calculate(!m_digitalInputTop.get());
    // m_hasFirstBall = m_debouncerColorTop.calculate(isTopBlue() || isTopRed());
    // m_hasSecondBall = m_debouncerColorBottom.calculate(isBottomBlue() || isdBottomRed());
    SmartDashboard.putBoolean("ball-bottom", m_isBallAtBottom);
    SmartDashboard.putBoolean("ball-top", m_isballAtTop);
    SmartDashboard.putNumber("ball count", m_ballCount);
    // SmartDashboard.putBoolean("Top Red", isTopRed());
    // SmartDashboard.putBoolean("Top Blue", isTopBlue());
    // SmartDashboard.putBoolean("Bottom Red", isBottomRed());
    // SmartDashboard.putBoolean("Bottom Blue", isBottomBlue());
    // SmartDashboard.putBoolean("is-top-red", isBallRed(getColorTop(), getColorTopProximity()));
    // SmartDashboard.putBoolean("is-top-blue", isBallBlue(getColorTop(), getColorTopProximity()));
    // SmartDashboard.putBoolean("is-bottom-red", isBallRed(getColorBottom(), getColorBottomProximity()));
    // SmartDashboard.putBoolean("is-bottom-blue", isBallBlue(getColorBottom(), getColorBottomProximity()));
  }
}
