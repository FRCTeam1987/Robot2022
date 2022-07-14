// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiAuto extends SequentialCommandGroup {
  /** Creates a new FiveBallAuto. */
  private double m_timeToWait = 2;

  public TaxiAuto(final XboxController controller, final DrivetrainSubsystem drivetrainSubsystem, final RobotContainer robotContainer) {
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TaxiAuto(controller, drivetrainSubsystem, robotContainer, m_timeToWait)
    );
  }

  public TaxiAuto(final XboxController controller, final DrivetrainSubsystem drivetrainSubsystem, final RobotContainer robotContainer, double timeToWait) {
    m_timeToWait = timeToWait;
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(m_timeToWait),
      drivetrainSubsystem.followPathCommand(true, "Taxi"),
      robotContainer.shootCommandHelper()
    );
  }
}
