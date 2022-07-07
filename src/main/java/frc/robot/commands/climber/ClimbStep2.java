// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.drivetrain.WaitUntilRoll;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbStep2 extends ParallelCommandGroup {
  /** Creates a new ClimberGroundToMedium. */
  private final TelescopeSubsystem m_telescopeFront;
  private final TelescopeSubsystem m_telescopeBack;
  private final DrivetrainSubsystem m_drivetrain;

  public ClimbStep2(TelescopeSubsystem telescopeFront, TelescopeSubsystem telescopeBack, DrivetrainSubsystem drivetrain) {
    m_telescopeFront = telescopeFront;
    m_telescopeBack = telescopeBack;
    m_drivetrain = drivetrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ArmGoToPosition(m_climber, ClimberArm.kFront, 3.5, 0.7)
      // new ArmGoToPosition(m_telescopeFront, m_telescopeBack, 3.5, 3.5)
      new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_drivetrain.zeroRoll();
          m_telescopeFront.setVoltageSaturation(6.75);
        }),
        new TelescopeGoToClosedLoop(m_telescopeFront, TelescopeSubsystem.k_minExtensionTicks - 2000, true), //~0 inch
        new InstantCommand(() -> m_telescopeFront.setVoltageSaturation())
      ),
      new SequentialCommandGroup(
        new TelescopeGoToClosedLoop(m_telescopeBack, TelescopeSubsystem.k_maxBackExtensionTicks - 20000),
        // new WaitUntilRoll(m_drivetrain, false, -38),
        new WaitCommand(0.75), //roll -37
        new InstantCommand(() -> System.out.println("Climb2 1 Roll with offset: " + m_drivetrain.getRollWithOffset())),
        // new InstantCommand(() -> m_drivetrain.getRollWithOffset()),
        new TelescopeGoToClosedLoop(m_telescopeBack, TelescopeSubsystem.k_maxBackExtensionTicks + 2000),  // long arm max extension ticks
        new InstantCommand(() -> System.out.println("Climb2 2 Roll with offset: " + m_drivetrain.getRollWithOffset()))
      )
    );
  }
}