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
public class TelescopeToHome extends SequentialCommandGroup {
  private final TelescopeSubsystem m_telescope;

  /** Creates a new ClimberToHome. */
  public TelescopeToHome(TelescopeSubsystem telescope) { //TODO Change to wait for current spike then stop climber
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_telescope = telescope;
    addCommands(
      new InstantCommand(() -> m_telescope.retract()),
      new WaitUntilCommand(() -> Math.abs(m_telescope.getPositionInches()) < 0.5 || Math.abs(m_telescope.getCurrent()) > 10),
      new InstantCommand(() -> m_telescope.stopTelescope()),
      new InstantCommand(() -> m_telescope.zeroTelescope())
    );
  }


@Override
public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    m_telescope.stopTelescope();
  }
}