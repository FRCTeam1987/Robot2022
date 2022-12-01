// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterRpm extends CommandBase {

  private final ShooterSubsystem m_shooter;
  private final DoubleSupplier m_rpmSupplier;
  private final Debouncer m_rpmDebouncer;
  private boolean m_isAtRpm;

  /** Creates a new SetShooterRpm. */
  public SetShooterRpm(final ShooterSubsystem shooter, final DoubleSupplier rpmSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = shooter;
    m_rpmSupplier = rpmSupplier;
    m_rpmDebouncer = new Debouncer(0.04);
    m_isAtRpm = false;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final double desiredRpm = m_rpmSupplier.getAsDouble();
    if (desiredRpm == 0.0) {
      m_shooter.stop();
    } else {
      m_shooter.setRPM(m_rpmSupplier.getAsDouble() + m_shooter.getOffsetRPM());
    }
    m_isAtRpm = m_rpmDebouncer.calculate(false);
    SmartDashboard.putNumber("Shooter RPM Goal", desiredRpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double desiredRpm = m_rpmSupplier.getAsDouble();
    if (desiredRpm == 0.0) {
      return true;
    }
    m_isAtRpm = m_rpmDebouncer.calculate(Util.isWithinTolerance(m_shooter.getRPM(), desiredRpm + m_shooter.getOffsetRPM(), Constants.Shooter.Shooter_RPM_Tolerance));
    return m_isAtRpm;
  }
}
