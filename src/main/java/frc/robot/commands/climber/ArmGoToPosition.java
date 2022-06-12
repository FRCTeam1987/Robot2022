// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ArmGoToPosition extends CommandBase {
  /** Creates a new ArmGoToPosition. */
  private final Enum m_arm;
  private final double m_desiredPosition;
  private final ClimberSubsystem m_climber;
  private double m_percentSpeed;

  public ArmGoToPosition(ClimberSubsystem climberSubsystem, Enum climberArm, double desiredPosition, double percentSpeed) { //remove enum and create 2 doubles for desired position (front and back)
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = climberArm;
    m_desiredPosition = desiredPosition;
    m_climber = climberSubsystem;
    m_percentSpeed = percentSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //set front arm and back arm to position (parellel)
    if (m_arm == ClimberSubsystem.ClimberArm.kFront) {
      double percentSign = m_climber.getFrontPosition() > m_desiredPosition ? -1 : 1;
      m_percentSpeed = Math.copySign(m_percentSpeed, percentSign);
      new SequentialCommandGroup(
          new InstantCommand(() -> m_climber.climberFrontExtend(m_percentSpeed)),
          new WaitUntilCommand(() -> Math.abs(m_climber.getFrontPosition()) > Constants.Climber.initialExtendPosition), //20
          new InstantCommand(() -> m_climber.climberFrontStop())
        );
    } else {
      double percentSign = m_climber.getBackPosition() > m_desiredPosition ? -1 : 1;
      m_percentSpeed = Math.copySign(m_percentSpeed, percentSign);
      new SequentialCommandGroup(
          new InstantCommand(() -> m_climber.climberBackExtend(m_percentSpeed)),
          new WaitUntilCommand(() -> Math.abs(m_climber.getBackPosition()) > Constants.Climber.initialExtendPosition), //20
          new InstantCommand(() -> m_climber.climberBackStop())
      );
    }

    /**
     * if armFront then
     *  
     *  setArmFrontMotor(-0.8 or 0.8)
     * if armBack then
     *  setArmBackMotor(...)
     */
    // double currentPos = 10;
    // double targetPos = 20;
    // double percentSign = currentPos > targetPos ? -1 : 1
    // Math.copySign(0.8, percentSign);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.climberFrontStop();
    m_climber.climberBackStop();
    // stop motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false; // isWithinTolerance
    if (m_arm == ClimberSubsystem.ClimberArm.kFront) {
      if (Math.abs(m_climber.getFrontPosition() - m_desiredPosition) <= 3) {
        return true;
      }
    } else {
      if (Math.abs(m_climber.getBackPosition() - m_desiredPosition) <= 3) {
        return true;
      }
    }
    return false;
  }
}
