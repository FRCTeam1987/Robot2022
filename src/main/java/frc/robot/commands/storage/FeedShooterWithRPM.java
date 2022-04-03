// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.storage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;


public class FeedShooterWithRPM extends CommandBase {
  
  private final ShooterSubsystem m_shooter;
  private final StorageSubsystem m_storage;
  private boolean m_hasStartedShooting;

  /** Creates a new FeedShooterWithRPM. */
  public FeedShooterWithRPM(StorageSubsystem storageSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooterSubsystem;
    m_storage = storageSubsystem;
    addRequirements(m_storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hasStartedShooting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Util.isWithinTolerance(m_shooter.getRpmSetpointError(), 0, Constants.Shooter.Shooter_RPM_Tolerance * Constants.Shooter.SHOOTER_REDUCTION)) {
      m_storage.runForShooter();
      m_hasStartedShooting = true;
    } else if (m_hasStartedShooting) {
      m_storage.runForIntake();
    } else {
      m_storage.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_storage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
