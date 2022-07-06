// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.WaitUntilRoll;
import frc.robot.subsystems.ClimberBackSubsystem;
import frc.robot.subsystems.ClimberFrontSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbStep3 extends ParallelCommandGroup {

  private final TelescopeSubsystem m_telescopeFront;
  private final TelescopeSubsystem m_telescopeBack;
  private final DrivetrainSubsystem m_drivetrain;

  /** Creates a new ClimberShift. */
  public ClimbStep3(TelescopeSubsystem frontTelescope, TelescopeSubsystem backTelescope, DrivetrainSubsystem drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_telescopeFront = frontTelescope;
    m_telescopeBack = backTelescope;
    m_drivetrain = drivetrain;
    addCommands(
      // new TelescopeGoToClosedLoop(frontTelescope, TelescopeSubsystem.k_maxExtensionTicks),
      // new TelescopeGoToClosedLoop(backTelescope, 6000)

      new SequentialCommandGroup(
        // new InstantCommand(() -> m_telescopeFront.setVoltageSaturation(9)),
        new TelescopeGoToClosedLoop(m_telescopeFront, 50000, true), //~0 inch
        // new WaitCommand(0.5),
        new WaitCommand(0.3),
        new TelescopeGoToClosedLoop(m_telescopeFront, TelescopeSubsystem.k_maxExtensionTicks - 20000, true), //~0 inch
        new WaitCommand(0.2),

        new TelescopeGoToClosedLoop(m_telescopeFront, TelescopeSubsystem.k_maxExtensionTicks),
        new WaitCommand(0.15),
        new TelescopeGoToClosedLoop(m_telescopeFront, TelescopeSubsystem.k_maxExtensionTicks - 5000)
        // new InstantCommand(() -> m_telescopeFront.setVoltageSaturation())
      ),
      new SequentialCommandGroup(
        // new WaitCommand(0.2),
        new TelescopeGoToClosedLoop(m_telescopeBack, TelescopeSubsystem.k_minExtensionTicks + 1000)
      )
      // new SequentialCommandGroup(
      //   new WaitCommand(0.5),
      //   new TelescopeGoToClosedLoop(m_telescopeFront, TelescopeSubsystem.k_maxExtensionTicks / 2),
      //   // new WaitUntilRoll(m_drivetrain, false, -38),
      //   new WaitCommand(0.5), //roll -37
      //   new InstantCommand(() -> System.out.println("Climb3 1 Roll with offset: " + m_drivetrain.getRollWithOffset())),
      //   // new InstantCommand(() -> m_drivetrain.getRollWithOffset()),
      //   new TelescopeGoToClosedLoop(m_telescopeFront, 227000),  // short arm max extend
      //   new InstantCommand(() -> System.out.println("Climb3 2 Roll with offset: " + m_drivetrain.getRollWithOffset()))
      // )
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
