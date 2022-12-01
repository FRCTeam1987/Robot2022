// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.PolynomialRegression;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwerveCharacterizationFF extends CommandBase {
  private static final double startDelaySecs = 2.0;
  private static final double rampRateVoltsPerSec = 0.25;

  private final FeedForwardCharacterizationData dataFL;
  private final FeedForwardCharacterizationData dataFR;
  private final FeedForwardCharacterizationData dataBL;
  private final FeedForwardCharacterizationData dataBR;

  private final boolean m_isForwards;
  private final boolean m_isRotation;
  private final DrivetrainSubsystem m_drivetrain;

  private final Timer timer = new Timer();

  /** Creates a new SwerveCharacterization. */
  public SwerveCharacterizationFF(final DrivetrainSubsystem drive, final boolean isForwards, final boolean isRotation) {
    m_drivetrain = drive;
    m_isForwards = isForwards;
    m_isRotation = isRotation;
    addRequirements(m_drivetrain);

    dataFL = new FeedForwardCharacterizationData("FL");
    dataFR = new FeedForwardCharacterizationData("FR");
    dataBL = new FeedForwardCharacterizationData("BL");
    dataBR = new FeedForwardCharacterizationData("BR");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dataFL.reset();
    dataFR.reset();
    dataBL.reset();
    dataBR.reset();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < startDelaySecs) {
      m_drivetrain.drive(new ChassisSpeeds());
    } else {
      double voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec * (m_isForwards ? 1 : -1);
      m_drivetrain.driveVolts(voltage, m_isRotation);
      dataFL.add(m_drivetrain.getVelocityFL(), voltage);
      dataFR.add(m_drivetrain.getVelocityFR(), voltage);
      dataBL.add(m_drivetrain.getVelocityBL(), voltage);
      dataBR.add(m_drivetrain.getVelocityBR(), voltage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    m_drivetrain.drive(new ChassisSpeeds());
    dataFL.print();
    dataFR.print();
    dataBL.print();
    dataBR.print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class FeedForwardCharacterizationData {
    private final String name;
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public FeedForwardCharacterizationData(String name) {
      this.name = name;
    }

    public void add(double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void reset() {
      velocityData.clear();
      voltageData.clear();
    }

    public void print() {
      PolynomialRegression regression = new PolynomialRegression(
          velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
          voltageData.stream().mapToDouble(Double::doubleValue).toArray(), 1);

      System.out.println("FF Characterization Results (" + name + "):");
      System.out
          .println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tR2=%.5f", regression.R2()));
      System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
      System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
    }
  }
}
