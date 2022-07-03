// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberBackSubsystem;
import frc.robot.subsystems.ClimberFrontSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ClimberArmExtend extends CommandBase {

  private final ClimberFrontSubsystem m_climberFront;
  private final ClimberBackSubsystem m_climberBack;
  private final DrivetrainSubsystem m_drivetrain;
  private final double m_extendLength;

  /** Creates a new ClimberArmExtend. */
  public ClimberArmExtend(ClimberFrontSubsystem climberFrontSubsystem, ClimberBackSubsystem climberBackSubsystem, DrivetrainSubsystem drivetrainSubsystem, double extendLength) {
    m_climberFront = climberFrontSubsystem;
    m_climberBack = climberBackSubsystem;
    m_drivetrain = drivetrainSubsystem;
    m_extendLength = extendLength;
    addRequirements(m_climberFront, m_climberBack);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double pitch = m_drivetrain.getPitch();
    if (pitch <= Constants.Climber.CLIMBER_MAX_EXTEND_ANGLE) {
      if (Math.abs(m_climberFront.getPosition()) < m_extendLength) {
        m_climberFront.climberExtend(0.75);
      } else {
        m_climberFront.climberStop();  
      }
      if (Math.abs(m_climberBack.getPosition()) < m_extendLength) {
        m_climberBack.climberExtend(0.75);
      } else {
        m_climberBack.climberStop();
      }
    } else {
      m_climberFront.climberStop();
      m_climberBack.climberStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberFront.climberStop();
    m_climberBack.climberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_climberFront.getPosition()) > m_extendLength) && (Math.abs(m_climberBack.getPosition()) > m_extendLength);
  }
}
