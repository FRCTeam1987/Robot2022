// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ClimberArmExtend extends CommandBase {

  private final ClimberSubsystem m_climber;
  private final DrivetrainSubsystem m_drivetrain;
  private boolean finished = false;
  private final double m_extendLength;

  /** Creates a new ClimberArmExtend. */
  public ClimberArmExtend(ClimberSubsystem climberSubsystem, DrivetrainSubsystem drivetrainSubsystem, double extendLength) {
    m_climber = climberSubsystem;
    m_drivetrain = drivetrainSubsystem;
    m_extendLength = extendLength;
    addRequirements(m_climber);
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
      if (Math.abs(m_climber.getRightPosition()) < m_extendLength) {
        m_climber.climberRightExtend(0.75);
      } else {
        m_climber.climberRightStop();  
      }
      if (Math.abs(m_climber.getLeftPosition()) < m_extendLength) {
        m_climber.climberLeftExtend(0.75);
      } else {
        m_climber.climberLeftStop();
      }
    } else {
      m_climber.climberRightStop();
      m_climber.climberLeftStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.climberRightStop();
    m_climber.climberLeftStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_climber.getRightPosition()) > m_extendLength) && (Math.abs(m_climber.getLeftPosition()) > m_extendLength);
  }
}
