// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeGoToClosedLoop extends CommandBase {

  private final TelescopeSubsystem m_telescope;
  private final int m_desiredPosition;
  private final boolean m_shouldLogPosition;

  public TelescopeGoToClosedLoop(final TelescopeSubsystem telescope, final int desiredPosition) {
    this(telescope, desiredPosition, false);
  }

  /**
   * Creates a new TelescopeGoToClosedLoop.
   * @param telescope The telescope to control.
   * @param desiredPosition The desired position in ticks.
   */
  public TelescopeGoToClosedLoop(final TelescopeSubsystem telescope, final int desiredPosition, final boolean shouldLogPosition) {
    m_telescope = telescope;
    m_desiredPosition = desiredPosition;
    m_shouldLogPosition = shouldLogPosition;
    addRequirements(m_telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_telescope.hasZeroed() == false) {
      return;
    }
    m_telescope.setPositionTicks(m_desiredPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shouldLogPosition) {
      System.out.println(" Arm position: " + m_telescope.getPositionTicks() + " Current: " + m_telescope.getCurrent());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_telescope.stopTelescope();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_telescope.hasZeroed() == false) {
      DriverStation.reportWarning("Telescope - Go To Position - Not zeroed!", false);
      return true;
    }
    // if (m_telescope.getCurrent() > 50) {
    //   DriverStation.reportWarning("Telescope - Go To Position - Current Spiked!", false);
    //   return true;
    // }
    return Util.isWithinTolerance(m_telescope.getPositionTicks(), m_desiredPosition, 10000);
  }
}
