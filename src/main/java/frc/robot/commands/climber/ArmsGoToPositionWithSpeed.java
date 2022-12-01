// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Util;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmsGoToPositionWithSpeed extends ParallelCommandGroup {
  private final double m_desiredFrontPosition;
  private final double m_desiredBackPosition;
  private final TelescopeSubsystem m_telescopeFront;
  private final TelescopeSubsystem m_telescopeBack;
  private final double m_percentSpeedFront;
  private final double m_percentSpeedBack;
  // private double m_percentSpeed;
  /** Creates a new ArmsGoToPosition. */
  public ArmsGoToPositionWithSpeed(TelescopeSubsystem telescopeFront, TelescopeSubsystem telescopeBack, double desiredFrontPosition, double desiredBackPosition, double percentSpeedFront, double percentSpeedBack) {
    m_desiredFrontPosition = desiredFrontPosition;
    m_desiredBackPosition = desiredBackPosition;
    m_telescopeFront = telescopeFront;
    m_telescopeBack = telescopeBack;
    m_percentSpeedFront = percentSpeedFront;
    m_percentSpeedBack = percentSpeedBack;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new InstantCommand(() -> m_telescopeFront.stopTelescope()), 
        new ConditionalCommand(
          new TelescopeRetract(m_telescopeFront, desiredFrontPosition, m_percentSpeedFront), //TODO needs to split subsystems so that commands dont intersect
          new TelescopeExtend(m_telescopeFront, desiredFrontPosition, m_percentSpeedFront), 
          () -> m_telescopeFront.getPositionInches() > m_desiredFrontPosition
        ), 
        () -> Util.isWithinTolerance(Math.abs(m_telescopeFront.getPositionInches()), desiredFrontPosition, 1)
      ),
      new ConditionalCommand(
        new InstantCommand(() -> m_telescopeBack.stopTelescope()), 
        new ConditionalCommand(
          new TelescopeRetract(m_telescopeBack, desiredBackPosition, m_percentSpeedBack),
          new TelescopeExtend(m_telescopeBack, desiredBackPosition, m_percentSpeedBack), 
          () -> m_telescopeBack.getPositionInches() > m_desiredBackPosition
        ), 
        () -> Util.isWithinTolerance(Math.abs(m_telescopeBack.getPositionInches()), desiredBackPosition, 1)
      )
    );
  }
}
