// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class ClimberArmsExtend extends CommandBase {

  private final TelescopeSubsystem m_frontTelescope;
  private final TelescopeSubsystem m_backTelescope;
  private final DrivetrainSubsystem m_drivetrain;
  private final double m_extendLength;

  /** Creates a new ClimberArmExtend. */
  public ClimberArmsExtend(TelescopeSubsystem frontTelescope, TelescopeSubsystem backTelescope, DrivetrainSubsystem drivetrainSubsystem, double extendLength) {
    m_frontTelescope = frontTelescope;
    m_backTelescope = backTelescope;
    m_drivetrain = drivetrainSubsystem;
    m_extendLength = extendLength;
    addRequirements(m_frontTelescope, m_backTelescope);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double pitch = m_drivetrain.getPitch();
    if (pitch <= Constants.Climber.CLIMBER_MAX_EXTEND_ANGLE) {
      if (Math.abs(m_frontTelescope.getPositionInches()) < m_extendLength) {
        m_frontTelescope.extend(0.75);
      } else {
        m_frontTelescope.stopTelescope();  
      }
      if (Math.abs(m_backTelescope.getPositionInches()) < m_extendLength) {
        m_backTelescope.extend(0.75);
      } else {
        m_backTelescope.stopTelescope();
      }
    } else {
      m_frontTelescope.stopTelescope();
      m_backTelescope.stopTelescope();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_frontTelescope.stopTelescope();
    m_backTelescope.stopTelescope();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_frontTelescope.getPositionInches()) > m_extendLength) && (Math.abs(m_backTelescope.getPositionInches()) > m_extendLength);
  }
}
