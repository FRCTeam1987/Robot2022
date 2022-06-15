// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Util;
import frc.robot.subsystems.ClimberBackSubsystem;
import frc.robot.subsystems.ClimberFrontSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmsGoToPosition extends ParallelCommandGroup {
  private final double m_desiredFrontPosition;
  private final double m_desiredBackPosition;
  private final ClimberFrontSubsystem m_climberFront;
  private final ClimberBackSubsystem m_climberBack;
  private final double m_percentSpeedFront;
  private final double m_percentSpeedBack;
  private double m_percentSpeed;
  /** Creates a new ArmsGoToPosition. */
  public ArmsGoToPosition(ClimberFrontSubsystem climberFrontSubsystem, ClimberBackSubsystem climberBackSubsystem, double desiredFrontPosition, double desiredBackPosition, double percentSpeedFront, double percentSpeedBack) {
    m_desiredFrontPosition = desiredFrontPosition;
    m_desiredBackPosition = desiredBackPosition;
    m_climberFront = climberFrontSubsystem;
    m_climberBack = climberBackSubsystem;
    m_percentSpeedFront = percentSpeedFront;
    m_percentSpeedBack = percentSpeedBack;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new InstantCommand(() -> m_climberFront.climberStop()), 
        new ConditionalCommand(
          new ClimberFrontRetract(m_climberFront, desiredFrontPosition, m_percentSpeedFront), //TODO needs to split subsystems so that commands dont intersect
          new ClimberFrontExtend(m_climberFront, desiredFrontPosition, m_percentSpeedFront), 
          () -> m_climberFront.getPosition() > m_desiredFrontPosition
        ), 
        () -> Util.isWithinTolerance(Math.abs(m_climberFront.getPosition()), desiredFrontPosition, 1)
      ),
      new ConditionalCommand(
        new InstantCommand(() -> m_climberBack.climberStop()), 
        new ConditionalCommand(
          new ClimberBackRetract(m_climberBack, desiredBackPosition, m_percentSpeedBack),
          new ClimberBackExtend(m_climberBack, desiredBackPosition, m_percentSpeedBack), 
          () -> m_climberBack.getPosition() > m_desiredBackPosition
        ), 
        () -> Util.isWithinTolerance(Math.abs(m_climberBack.getPosition()), desiredBackPosition, 1)
      )
    );
  }
}
