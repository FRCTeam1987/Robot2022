// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeAutoHome extends CommandBase {

  private final TelescopeSubsystem m_telescope;

  /** Creates a new TelescopeAutoHome. */
  public TelescopeAutoHome(final TelescopeSubsystem telescope) {
    m_telescope = telescope;
    addRequirements(m_telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_telescope.isBottomedOut()) {
      return;
    }
    m_telescope.retract();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_telescope.stopTelescope();
    m_telescope.zeroTelescope();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_telescope.isBottomedOut() || m_telescope.getCurrent() > 9.5;
  }
}
