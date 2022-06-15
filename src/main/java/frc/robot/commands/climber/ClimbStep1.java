// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberBackSubsystem;
import frc.robot.subsystems.ClimberFrontSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ClimberFrontSubsystem.ClimberArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbStep1 extends SequentialCommandGroup {
  /** Creates a new ClimberGroundToMedium. */
  private final ClimberFrontSubsystem m_climberFront;
  private final ClimberBackSubsystem m_climberBack;

  public ClimbStep1(ClimberFrontSubsystem climberFrontSubsystem, ClimberBackSubsystem climberBackSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    m_climberFront = climberFrontSubsystem;
    m_climberBack = climberBackSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmGoToPosition(m_climberFront, m_climberBack, 20, 0.5)
    );
  }
}

// new SequentialCommandGroup(
//           new InstantCommand(() -> new ArmGoToPosition(m_climber, ClimberArm.kFront, 3.5, 0.7)),
//           new WaitUntilCommand(() -> Math.abs(climberSubsystem.getFrontPosition()) < 3.5),
//           new InstantCommand(() -> climberSubsystem.climberFrontStop())
//         )