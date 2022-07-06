// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbStep1 extends ParallelCommandGroup {
  /** Creates a new ClimberGroundToMedium. */
  private final TelescopeSubsystem m_telescopeFront;

  public ClimbStep1(TelescopeSubsystem telescopeFront) {
    m_telescopeFront = telescopeFront;
    // m_telscopeBack = telescopBack;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TelescopeGoToClosedLoop(m_telescopeFront, 209000, true) // short telescope reach first rung
      // new ArmGoToPosition(m_telescopeFront, m_telscopeBack, 20, 0.5)
    );
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      if (interrupted) {
        System.out.println("Climb step 1 Interupted");
        m_telescopeFront.stopTelescope();
      }
  }

}

// new SequentialCommandGroup(
//           new InstantCommand(() -> new ArmGoToPosition(m_climber, ClimberArm.kFront, 3.5, 0.7)),
//           new WaitUntilCommand(() -> Math.abs(climberSubsystem.getFrontPosition()) < 3.5),
//           new InstantCommand(() -> climberSubsystem.climberFrontStop())
//         )