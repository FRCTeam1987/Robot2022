// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import  static frc.robot.Constants.Climber.*;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberRetract extends SequentialCommandGroup {

  private final ClimberSubsystem m_climber;


  /** Creates a new ClimberPullUp. */
  public ClimberRetract(ClimberSubsystem climberSubsystem, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climber = climberSubsystem;
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> climberSubsystem.climberFrontRetract(speed)), // speed = 0.75
          new WaitUntilCommand(() -> Math.abs(climberSubsystem.getFrontPosition()) < CLIMBER_LOWER_TOLERANCE),
          new InstantCommand(() -> climberSubsystem.climberFrontStop())
        ),
        new SequentialCommandGroup(
          new InstantCommand(() -> climberSubsystem.climberBackRetract(speed)), // speed = 0.75
          new WaitUntilCommand(() -> Math.abs(climberSubsystem.getBackPosition()) < CLIMBER_LOWER_TOLERANCE),
          new InstantCommand(() -> climberSubsystem.climberBackStop())  
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
