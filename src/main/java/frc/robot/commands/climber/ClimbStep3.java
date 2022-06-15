// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberBackSubsystem;
import frc.robot.subsystems.ClimberFrontSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbStep3 extends SequentialCommandGroup {

  private final ClimberFrontSubsystem m_climberFront;
  private final ClimberBackSubsystem m_climberBack;

  /** Creates a new ClimberShift. */
  public ClimbStep3(ClimberFrontSubsystem climberFrontSubsystem, ClimberBackSubsystem climberBackSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climberFront = climberFrontSubsystem;
    m_climberBack = climberBackSubsystem;
    addCommands(
      // new ParallelCommandGroup(
        // new SequentialCommandGroup(
        //   new WaitCommand(1), //REMOVE ME WHEN TESTED, SAFTEY VALUE
        //   new InstantCommand(() -> climberSubsystem.climberFrontRetract(0.65)),
        //   new WaitUntilCommand(() -> Math.abs(climberSubsystem.getFrontPosition()) < 2),
        //   new InstantCommand(() -> climberSubsystem.climberFrontStop())
        // ),
        new ArmGoToPosition(m_climberFront, m_climberBack, 3.5, 8),
      // ),
      new WaitCommand(1),
      new ArmGoToPosition(m_climberFront, m_climberBack, 3.5, 3.5)

      // new ClimberArmExtend(climberSubsystem, drivetrainSubsystem, 24) //24 //TODO Check me


      // new ParallelCommandGroup(
      //   new SequentialCommandGroup(
      //     new InstantCommand(() -> climberSubsystem.climberFrontExtend(0.5)),
      //     new WaitUntilCommand(() -> Math.abs(climberSubsystem.getFrontPosition()) > 24),
      //     new InstantCommand(() -> climberSubsystem.climberFrontStop())
      //   ),
      //   new SequentialCommandGroup(
      //     new InstantCommand(() -> climberSubsystem.climberBackExtend(0.5)),
      //     new WaitUntilCommand(() -> Math.abs(climberSubsystem.getBackPosition()) > 24),
      //     new InstantCommand(() -> climberSubsystem.climberBackStop())
      //   )
      // )
    );
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      if (interrupted) {
        m_climberFront.climberStop();
        m_climberBack.climberStop();
      }
  }
}
