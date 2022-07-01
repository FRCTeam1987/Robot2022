// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class WaitUntilRoll extends CommandBase {
  /** Creates a new WaitUntilRoll. */
  private final DrivetrainSubsystem m_drivetrain;
  private final double m_desiredRoll;
  private final boolean m_waitUntilValueGreater;

  public WaitUntilRoll(DrivetrainSubsystem drivetrain, boolean waitUntilValueGreater, double desiredRoll) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_desiredRoll = desiredRoll;
    m_waitUntilValueGreater = waitUntilValueGreater;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setRollOffset();  // TODO Don't do this
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new WaitUntilCommand(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_waitUntilValueGreater == true) {
      return m_drivetrain.getRollWithOffset() > m_desiredRoll;
    } else if (m_waitUntilValueGreater == false) {
      return m_drivetrain.getRollWithOffset() < m_desiredRoll;
    } else {
      return false;
    }
  }
}
