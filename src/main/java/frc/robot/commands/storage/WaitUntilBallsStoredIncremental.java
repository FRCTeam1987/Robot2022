// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.StorageSubsystem;

import static frc.robot.Constants.Storage.*;

public class WaitUntilBallsStoredIncremental extends CommandBase {

  private final StorageSubsystem m_storage;
  private final int m_desiredBallCount;
  private boolean m_hadBallAtEntrance;
  private boolean m_hadBallAtExit;
  private boolean m_hasFirstBall;
  private boolean m_hasSecondBall;
  private double m_colorTop;

  /** Creates a new WaitUntilMaxBallsStored. */
  public WaitUntilBallsStoredIncremental(final StorageSubsystem storage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(storage, MAX_BALL_COUNT);
  }

  public WaitUntilBallsStoredIncremental(final StorageSubsystem storage, final int desiredBallCount) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_storage = storage;
    m_desiredBallCount = desiredBallCount;
    m_hadBallAtEntrance = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hadBallAtEntrance = m_storage.isBallAtEntrance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final boolean hasBallAtEntrance = m_storage.isBallAtEntrance();
    if (m_hadBallAtEntrance == false && hasBallAtEntrance == true) {
      m_storage.incrementBallCount();
      System.out.println("incremented");
    }
    // System.out.println("m_hadBallAtEntrance = " +  m_hadBallAtEntrance + ", " + "hasBallAtEntrance = " + hasBallAtEntrance + ", " + "m_hadBallAtExit = " + m_hadBallAtExit + ", " + "hasBallAtExit = " + hasBallAtExit);
    m_hadBallAtEntrance = hasBallAtEntrance;

    // if (m_hasFirstBall == true && m_hasSecondBall == false) {
    //   m_storage.setBallCount(1);
    // }
    // if (m_hasFirstBall == true && m_hasSecondBall == true) {
    //   m_storage.setBallCount(2);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_storage.getBallCount() >= m_desiredBallCount;
  }
}
