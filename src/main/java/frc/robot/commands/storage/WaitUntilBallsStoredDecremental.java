// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.StorageSubsystem;

import static frc.robot.Constants.Storage.*;

public class WaitUntilBallsStoredDecremental extends CommandBase {

  private final StorageSubsystem m_storage;
  private final int m_desiredBallCount;
  private boolean m_hadBallAtExit;
  private boolean m_hasFirstBall;
  private boolean m_hasSecondBall;
  private double m_colorTop;

  /** Creates a new WaitUntilMaxBallsStored. */
  public WaitUntilBallsStoredDecremental(final StorageSubsystem storage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(storage, MAX_BALL_COUNT);
  }

  public WaitUntilBallsStoredDecremental(final StorageSubsystem storage, final int desiredBallCount) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_storage = storage;
    m_desiredBallCount = desiredBallCount;
    m_hadBallAtExit = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hadBallAtExit =  m_storage.isBallAtExit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final boolean hasBallAtExit = m_storage.isBallAtExit();
    if (m_hadBallAtExit == false && hasBallAtExit == true) {
      m_storage.decrementBallCount();
      System.out.println("decremented");
    }
    // System.out.println("m_hadBallAtEntrance = " +  m_hadBallAtEntrance + ", " + "hasBallAtEntrance = " + hasBallAtEntrance + ", " + "m_hadBallAtExit = " + m_hadBallAtExit + ", " + "hasBallAtExit = " + hasBallAtExit);
    m_hadBallAtExit =  hasBallAtExit;

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
