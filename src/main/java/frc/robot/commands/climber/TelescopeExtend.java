// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TelescopeExtend extends SequentialCommandGroup {

  private final TelescopeSubsystem m_telescope;

  /** Creates a new ClimberExtend. */
  public TelescopeExtend(TelescopeSubsystem telescope, double desiredPosition, double percentSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_telescope = telescope;
    addCommands(
      // TODO for all climber motion, do nothing if before end game period
      // make more reusable across all climber movements
      // for example:
      // new ConditionalCommand(
      //   new InstantCommand(),
      //   new InstantCommand(/* does some climber stuff*/),
      //   () -> DriverStation.getMatchTime() < 40
      // ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> m_telescope.extend(percentSpeed)),
          new WaitUntilCommand(() -> Math.abs(m_telescope.getPositionInches()) > desiredPosition), //20
          new InstantCommand(() -> m_telescope.stopTelescope())
        )
      )
    );
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      if (interrupted) {
        m_telescope.stopTelescope();
      }
  }
}
