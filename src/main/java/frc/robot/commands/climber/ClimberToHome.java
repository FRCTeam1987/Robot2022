// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberToHome extends ParallelCommandGroup {
  private final TelescopeSubsystem m_telescopeFront;
  private final TelescopeSubsystem m_telescopeBack;
  /** Creates a new ClimberToHome. */
  public ClimberToHome(TelescopeSubsystem telescopeFront, TelescopeSubsystem telescopeBack) {
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      m_telescopeFront = telescopeFront;
      m_telescopeBack = telescopeBack;
      addCommands(
  
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(() -> m_telescopeBack.retract()),
            new WaitUntilCommand(() -> Math.abs(m_telescopeBack.getPositionInches()) < 0.5 || Math.abs(m_telescopeBack.getCurrent()) > 10),
            new InstantCommand(() -> m_telescopeBack.stopTelescope()),
            new InstantCommand(() -> m_telescopeBack.zeroTelescope())
          ),
          new SequentialCommandGroup(
            new InstantCommand(() -> m_telescopeFront.retract()),
            new WaitUntilCommand(() -> Math.abs(m_telescopeFront.getPositionInches()) < 0.5 || Math.abs(m_telescopeFront.getCurrent()) > 10),
            new InstantCommand(() -> m_telescopeFront.stopTelescope()),
            new InstantCommand(() -> m_telescopeFront.zeroTelescope())
          )
        )
      );
  }
  
  
  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      m_telescopeFront.stopTelescope();
      m_telescopeBack.stopTelescope();
      }
  }