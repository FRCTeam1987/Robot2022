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
public class ClimbStep3HighBar extends ParallelCommandGroup {

  private final TelescopeSubsystem m_telescopeFront;
  private final TelescopeSubsystem m_telescopeBack;
  private final DrivetrainSubsystem m_drivetrain;

  /** Creates a new ClimberShift. */
  public ClimbStep3HighBar(TelescopeSubsystem frontTelescope, TelescopeSubsystem backTelescope, DrivetrainSubsystem drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_telescopeFront = frontTelescope;
    m_telescopeBack = backTelescope;
    m_drivetrain = drivetrain;
    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(() -> System.out.println("Step 3 Start Roll: " + m_drivetrain.getRollWithOffset())),
        new DisengageFrictionBrakeTelescope(m_telescopeFront),
        new TelescopeGoToClosedLoop(m_telescopeFront, TelescopeSubsystem.k_minExtensionTicks + 80000),
        new InstantCommand(() -> m_telescopeFront.stopTelescope()), 
        new EngageFrictionBrakeTelescope(m_telescopeFront),
        new InstantCommand(() -> System.out.println("Climb Step 3 1 finished: " + m_telescopeFront.getPositionTicks() + " current roll: " + m_drivetrain.getRoll()))
      ),
      new  SequentialCommandGroup(
        new DisengageFrictionBrakeTelescope(m_telescopeBack),
        new WaitCommand(0.1),
        new TelescopeGoToClosedLoop(m_telescopeBack, TelescopeSubsystem.k_maxBackExtensionTicks - 80000, true).withTimeout(1),
        new InstantCommand(() -> m_telescopeBack.stopTelescope()), 
        new EngageFrictionBrakeTelescope(m_telescopeBack)
      )
    );
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      m_telescopeFront.stopTelescope();
      m_telescopeFront.engageBrake();
      m_telescopeBack.stopTelescope();
      m_telescopeBack.engageBrake();
  }
}
