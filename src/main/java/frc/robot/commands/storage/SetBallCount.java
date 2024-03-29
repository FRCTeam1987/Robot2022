// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetBallCount extends InstantCommand {

  private final StorageSubsystem m_storage;
  private final int m_desiredBallCount;

  public SetBallCount(final StorageSubsystem storage, final int desiredBallCount) {
    m_storage = storage;
    m_desiredBallCount = desiredBallCount;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Setting ball count: " + m_desiredBallCount);
    m_storage.setBallCount(m_desiredBallCount);
  }

  @Override
  public boolean runsWhenDisabled() {
    // TODO Auto-generated method stub
    return true;
  }
}
