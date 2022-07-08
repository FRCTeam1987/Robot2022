// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EngageFrictionBrakeClimber extends InstantCommand {

  private final TelescopeSubsystem m_frontTelescope;
  private final TelescopeSubsystem m_backTelescope;

  public EngageFrictionBrakeClimber(TelescopeSubsystem frontTelescope, TelescopeSubsystem backTelescope) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_frontTelescope = frontTelescope;
    m_backTelescope = backTelescope;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_frontTelescope.stopTelescope();
    m_backTelescope.stopTelescope();
    m_frontTelescope.engageBrake();
    m_backTelescope.engageBrake();
  }
}
