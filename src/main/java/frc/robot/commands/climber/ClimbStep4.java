// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbStep4 extends ParallelCommandGroup {

  private final TelescopeSubsystem m_telescopeFront;
  private final TelescopeSubsystem m_telescopeBack;
  private final DrivetrainSubsystem m_drivetrain;

  /** Creates a new ClimberShift. */
  public ClimbStep4(TelescopeSubsystem frontTelescope, TelescopeSubsystem backTelescope, DrivetrainSubsystem drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_telescopeFront = frontTelescope;
    m_telescopeBack = backTelescope;
    m_drivetrain = drivetrain;
    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(() -> System.out.println("Step 4 Start Roll: " + m_drivetrain.getRollWithOffset())),
        // new TelescopeGoToClosedLoop(m_telescopeFront, TelescopeSubsystem.k_maxFrontExtensionTicks - 10000),
        new TelescopeRetract(m_telescopeFront, TelescopeSubsystem.k_maxFrontExtensionTicks - 25000, 1.0),
        new EngageFrictionBrakeTelescope(m_telescopeFront),
        new InstantCommand(() -> System.out.println("Climb Step 4 1 finished: " + m_telescopeFront.getPositionTicks() + " current roll: " + m_drivetrain.getRoll()))
      ),
      new  SequentialCommandGroup(
        // new WaitCommand(.2)
        new DisengageFrictionBrakeTelescope(m_telescopeBack),
        new WaitCommand(0.1),
        new TelescopeGoToClosedLoop(m_telescopeBack, TelescopeSubsystem.k_minExtensionTicks + 25000, true),
        new EngageFrictionBrakeTelescope(m_telescopeBack),
        new InstantCommand(() -> System.out.println("Climb Step 4 2 finished: " + m_telescopeFront.getPositionTicks() + " current roll: " + m_drivetrain.getRoll()))

      )
    );
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      if (interrupted) {
        m_telescopeFront.engageBrake();
        m_telescopeBack.engageBrake();
      }
  }
}
