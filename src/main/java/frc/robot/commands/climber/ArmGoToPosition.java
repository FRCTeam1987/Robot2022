// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopeSubsystem;

public class ArmGoToPosition extends CommandBase {
  /** Creates a new ArmGoToPosition. */
  private final double m_desiredFrontPosition;
  private final double m_desiredBackPosition;
  private final TelescopeSubsystem m_telescopeFront;
  private final TelescopeSubsystem m_telescopeBack;

  public ArmGoToPosition(TelescopeSubsystem telescopeFront, TelescopeSubsystem telescopeBack, double desiredFrontPosition, double desiredBackPosition) { //remove enum and create 2 doubles for desired position (front and back)
    // Use addRequirements() here to declare subsystem dependencies.
    m_desiredFrontPosition = desiredFrontPosition;
    m_desiredBackPosition = desiredBackPosition;
    m_telescopeFront = telescopeFront;
    m_telescopeBack = telescopeBack;
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

      if (m_telescopeFront.getPositionInches() < (m_desiredFrontPosition - (Constants.Climber.ExceptableErrorValue / 2))) {
        m_telescopeFront.extend();
      } else if (m_telescopeFront.getPositionInches() > (m_desiredFrontPosition + (Constants.Climber.ExceptableErrorValue / 2))) {
        m_telescopeFront.retract();
      } else {
        m_telescopeFront.stopTelescope();
      }


      if (m_telescopeBack.getPositionInches() < (m_desiredBackPosition - 0.5)) {
        m_telescopeBack.extend();
      } else if (m_telescopeBack.getPositionInches() > (m_desiredBackPosition + 0.5)) {
        m_telescopeBack.retract();
      } else {
        m_telescopeBack.stopTelescope();
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
    m_telescopeFront.stopTelescope();
    m_telescopeBack.stopTelescope();
    // stop motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false; // isWithinTolerance
      if ((Math.abs(m_telescopeFront.getPositionInches() - m_desiredFrontPosition) <= 3) && (Math.abs(m_telescopeBack.getPositionInches() - m_desiredBackPosition) <= 3)) {
        return true;
      }
    return false;
  }
}
