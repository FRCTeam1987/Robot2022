// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CollectorSubsystem;

import static frc.robot.Constants.Collector.*;

public class DeployCollector extends WaitCommand {

  private final CollectorSubsystem m_collector;

  /** Creates a new DeployCollector. */
  public DeployCollector(final CollectorSubsystem collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(COMMAND_DEPLOY_DURATION);
    m_collector = collector;
    addRequirements(m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    m_collector.deploy();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_collector.runRollerIn();
  }

}
