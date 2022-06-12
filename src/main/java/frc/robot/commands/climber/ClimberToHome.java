// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberToHome extends ParallelCommandGroup {
  private final ClimberSubsystem m_climber;

  /** Creates a new ClimberToHome. */
  public ClimberToHome(ClimberSubsystem climberSubsystem) { //TODO Change to wait for current spike then stop climber
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climber = climberSubsystem;
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> climberSubsystem.climberBackRetract()),
          new WaitUntilCommand(() -> Math.abs(climberSubsystem.getBackPosition()) < 0.5),
          new InstantCommand(() -> climberSubsystem.climberBackStop())
        ),
        new SequentialCommandGroup(
          new InstantCommand(() -> climberSubsystem.climberFrontRetract()),
          new WaitUntilCommand(() -> Math.abs(climberSubsystem.getFrontPosition()) < 0.5),
          new InstantCommand(() -> climberSubsystem.climberFrontStop())  
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