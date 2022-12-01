// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeGoToPosition extends CommandBase {
  private final double m_desiredPosition;
  private final TelescopeSubsystem m_telescope;
  /**
   * Creates a new TelescopeGoToPosition.
   * @param telescope The telescope to control.
   * @param desiredPosition The desired position in inches.
   */
  public TelescopeGoToPosition(TelescopeSubsystem telescope, double desiredPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_desiredPosition = desiredPosition;
    m_telescope = telescope;
    addRequirements(m_telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_telescope.hasZeroed() == false) {
      return;
    }
    if (m_telescope.getPositionInches() < (m_desiredPosition - (Climber.ExceptableErrorValue / 2))) {
      m_telescope.extend();
    } else if (m_telescope.getPositionInches() > (m_desiredPosition + (Climber.ExceptableErrorValue / 2))) {
      m_telescope.retract();
    } else {
      m_telescope.stopTelescope();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_telescope.stopTelescope();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_telescope.hasZeroed() == false || Math.abs(m_telescope.getPositionInches() - m_desiredPosition) <= Climber.ExceptableErrorValue;
  }
}
