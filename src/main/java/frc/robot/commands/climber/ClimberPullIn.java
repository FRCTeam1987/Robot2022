// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimberSubsystem;
import static frc.robot.Constants.Climber.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberPullIn extends SequentialCommandGroup {

  private final ClimberSubsystem m_climber;

  /** Creates a new ClimberPullIn. */
  public ClimberPullIn(ClimberSubsystem climberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climber = climberSubsystem;
    addCommands(
      new InstantCommand(() -> climberSubsystem.pivotUp(), climberSubsystem),
      new WaitCommand(1.0),
      new ParallelCommandGroup(
        new SequentialCommandGroup( /* Right Climber */
          new InstantCommand(() -> climberSubsystem.climberRightRetract(0.35 )),
          new WaitUntilCommand(() -> Math.abs(climberSubsystem.getRightPosition()) < CLIMBER_LOWER_TOLERANCE - .5),
          new InstantCommand(() -> climberSubsystem.climberRightStop())
        ),
        new SequentialCommandGroup( /* Left Climber */
          new InstantCommand(() -> climberSubsystem.climberLeftRetract(0.35)),
          new WaitUntilCommand(() -> Math.abs(climberSubsystem.getLeftPosition()) < CLIMBER_LOWER_TOLERANCE - .5),
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
