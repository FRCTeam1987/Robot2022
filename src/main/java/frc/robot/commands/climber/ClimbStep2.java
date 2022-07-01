// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbStep2 extends ParallelCommandGroup {
  /** Creates a new ClimberGroundToMedium. */
  private final TelescopeSubsystem m_telescopeFront;
  private final TelescopeSubsystem m_telescopeBack;

  public ClimbStep2(TelescopeSubsystem telescopeFront, TelescopeSubsystem telescopeBack) {
    m_telescopeFront = telescopeFront;
    m_telescopeBack = telescopeBack;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ArmGoToPosition(m_climber, ClimberArm.kFront, 3.5, 0.7)
      // new ArmGoToPosition(m_telescopeFront, m_telescopeBack, 3.5, 3.5)
      new TelescopeGoToPosition(m_telescopeFront, 3.5),
      new TelescopeGoToPosition(m_telescopeBack, 20)
    );
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      if (interrupted) {
        m_telescopeFront.stopTelescope();
        m_telescopeBack.stopTelescope();
      }
  }

}