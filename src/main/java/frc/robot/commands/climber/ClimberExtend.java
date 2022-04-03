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
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberExtend extends SequentialCommandGroup {

  private final ClimberSubsystem m_climber;

  /** Creates a new ClimberExtend. */
  public ClimberExtend(ClimberSubsystem climberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climber = climberSubsystem;
    addCommands(
      // TODO for all climber motion, do nothing if before end game period
      // make more reusable across all climber movements
      // for example:
      // new ConditionalCommand(
      //   new InstantCommand(),
      //   new InstantCommand(/* does some climber stuff*/),
      //   () -> DriverStation.getMatchTime() < 40
      // ),
      new InstantCommand(() -> climberSubsystem.unlock(), climberSubsystem),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> climberSubsystem.climberRightExtend(0.75)),
          new WaitUntilCommand(() -> Math.abs(climberSubsystem.getRightPosition()) > 20),
          new InstantCommand(() -> climberSubsystem.climberRightStop())
        ),
        new SequentialCommandGroup(
          new InstantCommand(() -> climberSubsystem.climberLeftExtend(0.75)),
          new WaitUntilCommand(() -> Math.abs(climberSubsystem.getLeftPosition()) > 20),
          new InstantCommand(() -> climberSubsystem.climberLeftStop())
        )
      )
    );
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      if (interrupted) {
        m_climber.climberStop();
      }
  }
}
