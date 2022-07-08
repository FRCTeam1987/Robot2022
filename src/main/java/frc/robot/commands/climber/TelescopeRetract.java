// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import  static frc.robot.Constants.Climber.*;

import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TelescopeRetract extends SequentialCommandGroup {

  private final TelescopeSubsystem m_telescope;


  /** Creates a new ClimberPullUp. */
  public TelescopeRetract(TelescopeSubsystem telescope, double desiredPosition, double percentSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_telescope = telescope;
    double m_telescopePositionTicks = m_telescope.getPositionTicks();
    addCommands(
      // new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> m_telescope.retract(percentSpeed)), // speed = 0.75
          new WaitUntilCommand(() -> m_telescopePositionTicks < desiredPosition || m_telescopePositionTicks <=  0),
          new InstantCommand(() -> m_telescope.stopTelescope())
        )
        // new SequentialCommandGroup(
        //   new InstantCommand(() -> climberSubsystem.climberBackRetract(speed)), // speed = 0.75
        //   new WaitUntilCommand(() -> Math.abs(climberSubsystem.getBackPosition()) < desiredPosition),
        //   new InstantCommand(() -> climberSubsystem.climberBackStop())  
        // )
      // )
    );
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      if (interrupted) {
        m_telescope.stopTelescope();
      } 
      if (m_telescope.getPositionTicks() <= 0) {
        DriverStation.reportWarning("Telescope - arm has gone below 0 ticks! arm: " + m_telescope, false);
        m_telescope.stopTelescope();
      }
  }
}
