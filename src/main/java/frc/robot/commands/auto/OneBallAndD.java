// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.storage.SetBallCount;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallAndD extends SequentialCommandGroup {
  /** Creates a new TwoBallAndDAuto. */
  public OneBallAndD(final XboxController controller, final DrivetrainSubsystem drivetrainSubsystem, final CollectorSubsystem collectorSubsystem, final StorageSubsystem storageSubsystem, final ShooterSubsystem shooterSubsystem, final LimeLight limelight, final RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //2BallAndDPart1
    addCommands(
      // new WaitCommand(4),
      new OneBallAndSteal(controller, drivetrainSubsystem, collectorSubsystem, storageSubsystem, shooterSubsystem, limelight, robotContainer),
      drivetrainSubsystem.followPathCommand(false, "1BallAndDPart3"),
      new InstantCommand(() -> {
        collectorSubsystem.deploy();
        collectorSubsystem.runRollerOut();
        storageSubsystem.runForOutput();
      }, collectorSubsystem, storageSubsystem),
      new WaitCommand(1.5),
      new InstantCommand(() -> {
        storageSubsystem.stop();
        collectorSubsystem.stop();
        collectorSubsystem.stow();
        controller.setRumble(RumbleType.kLeftRumble, 0);
        controller.setRumble(RumbleType.kRightRumble, 0);
      }, collectorSubsystem, storageSubsystem),
      new SetBallCount(storageSubsystem, 0),
      drivetrainSubsystem.followPathCommand(false, "1BallAndDPart4")
      // need to spit the ball to destage
      
    );
  }
}
