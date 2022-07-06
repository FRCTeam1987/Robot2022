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
  private final TelescopeSubsystem m_frontTelescope;
  private final TelescopeSubsystem m_backTelescope;

  /** Creates a new ClimberToHome. */
  public ClimberToHome(TelescopeSubsystem frontTelescope, TelescopeSubsystem backTelescope) { //TODO Change to wait for current spike then stop climber
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_frontTelescope = frontTelescope;
    m_backTelescope = backTelescope;
    addCommands(

      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> m_backTelescope.retract()),
          new WaitUntilCommand(() -> Math.abs(m_backTelescope.getPositionInches()) < 0.5),
          new InstantCommand(() -> m_backTelescope.stopTelescope())
        ),
        new SequentialCommandGroup(
          new InstantCommand(() -> m_frontTelescope.retract()),
          new WaitUntilCommand(() -> Math.abs(m_frontTelescope.getPositionInches()) < 0.5),
          new InstantCommand(() -> m_frontTelescope.stopTelescope())  
        )
      )
    );
  }


@Override
public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    if (interrupted) {
      m_backTelescope.stopTelescope();
      m_frontTelescope.stopTelescope();
    }
}
}