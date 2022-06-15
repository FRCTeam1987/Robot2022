// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberBackSubsystem;
import frc.robot.subsystems.ClimberFrontSubsystem;

public class ArmGoToPosition extends CommandBase {
  /** Creates a new ArmGoToPosition. */
  private final double m_desiredFrontPosition;
  private final double m_desiredBackPosition;
  private final ClimberFrontSubsystem m_climberFront;
  private final ClimberBackSubsystem m_climberBack;
  private double m_percentSpeedFront;
  private double m_percentSpeedBack;

  public ArmGoToPosition(ClimberFrontSubsystem climberFrontSubsystem, ClimberBackSubsystem climberBackSubsystem, double desiredFrontPosition, double desiredBackPosition) { //remove enum and create 2 doubles for desired position (front and back)
    // Use addRequirements() here to declare subsystem dependencies.
    m_desiredFrontPosition = desiredFrontPosition;
    m_desiredBackPosition = desiredBackPosition;
    m_climberFront = climberFrontSubsystem;
    m_climberBack = climberBackSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //set front arm and back arm to position (parellel)
    // if (front arm pos is below target) {
    //   go up();
    // } else if (front arm pos is above target) {
    //   go down();
    // } else {
    //   stop();
    // }

      if (m_climberFront.getPosition() < (m_desiredFrontPosition - (Constants.Climber.ExceptableErrorValue / 2))) {
        m_climberFront.climberExtend();
      } else if (m_climberFront.getPosition() > (m_desiredFrontPosition + (Constants.Climber.ExceptableErrorValue / 2))) {
        m_climberFront.climberRetract();
      } else {
        m_climberFront.climberStop();
      }


      if (m_climberBack.getPosition() < (m_desiredBackPosition - 0.5)) {
        m_climberBack.climberExtend();
      } else if (m_climberBack.getPosition() > (m_desiredBackPosition + 0.5)) {
        m_climberBack.climberRetract();
      } else {
        m_climberBack.climberStop();
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
    m_climberFront.climberStop();
    m_climberBack.climberStop();
    // stop motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false; // isWithinTolerance
      if ((Math.abs(m_climberFront.getPosition() - m_desiredFrontPosition) <= 3) && (Math.abs(m_climberBack.getPosition() - m_desiredBackPosition) <= 3)) {
        return true;
      }
    return false;
  }
}
