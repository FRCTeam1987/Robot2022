// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbStep4 extends ParallelCommandGroup {

  private final TelescopeSubsystem m_telescopeBack;

  /** Creates a new ClimberShift. */
  public ClimbStep4(TelescopeSubsystem telescopeBack) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_telescopeBack = telescopeBack;
    addCommands(
      
      // new TelescopeGoToPosition(telescopeFront, 20),
      new TelescopeGoToPosition(m_telescopeBack, 6)
    );
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      if (interrupted) {
        m_telescopeBack.stopTelescope();
      }
  }
}
