// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StorageSubsystem;
import static frc.robot.Constants.Storage.*;

public class WaitForBallShoot extends CommandBase {
  /** Creates a new WaitForBallShoot. */
  private final StorageSubsystem m_storage;
  private final int m_originalDesiredBallShootCount;
  private boolean m_hadBallAtExit;
  private int m_startBallCount;
  private int m_desiredBallShootCount;
  private boolean m_isBallAtTop;

  public WaitForBallShoot(final StorageSubsystem storage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(storage, true, MAX_BALL_COUNT);
  }
  public WaitForBallShoot(final StorageSubsystem storage, final int desiredBallShootCount) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(storage, true, desiredBallShootCount);
  }
  public WaitForBallShoot(final StorageSubsystem storage, final boolean isBallAtTop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(storage, isBallAtTop, MAX_BALL_COUNT);
  }
  public WaitForBallShoot(final StorageSubsystem storage, final boolean isBallAtTop, final int desiredBallShootCount) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_storage = storage;
    m_isBallAtTop = isBallAtTop;
    m_hadBallAtExit = false;
    m_startBallCount = 0;
    m_originalDesiredBallShootCount = desiredBallShootCount;
    m_desiredBallShootCount = desiredBallShootCount;
    // System.out.println("desiredBallShootCount in constructor = " + m_desiredBallShootCount);
    // m_desiredBallCount = desiredBallCount;
    // if (m_desiredBallCount < m_startBallCount) {
    //   m_desiredBallCount = m_startBallCount;
    // }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hadBallAtExit = isBallAtExit();
    m_startBallCount = m_storage.getBallCount();
    m_desiredBallShootCount  = Math.min(m_originalDesiredBallShootCount, m_startBallCount);
    // System.out.println("desiredBallShootCount in initialize = " + m_desiredBallShootCount);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final boolean hasBallAtExit = isBallAtExit();
    if (m_hadBallAtExit == false && hasBallAtExit == true) {
      m_storage.decrementBallCount();
    }
    m_hadBallAtExit = hasBallAtExit;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_storage.getBallCount() == 0 ||
      m_storage.getBallCount() == m_startBallCount-m_desiredBallShootCount;
  }

  public boolean isBallAtExit() {
    return m_isBallAtTop ? m_storage.isBallAtExit() : m_storage.isBallAtEntrance();
  }
}
