// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.CollectBalls;
import frc.robot.commands.shooter.EjectOneBallBottom;
import frc.robot.commands.storage.RunStorageIn;
import frc.robot.commands.storage.StopStorage;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallAndSteal extends SequentialCommandGroup {
  /** Creates a new TwoBallAndDAuto. */
  public OneBallAndSteal(final XboxController controller, final DrivetrainSubsystem drivetrainSubsystem, final CollectorSubsystem collectorSubsystem, final StorageSubsystem storageSubsystem, final ShooterSubsystem shooterSubsystem, final LimeLight limelight, final RobotContainer robotContainer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //2BallAndDPart1
    addCommands(
      new WaitCommand(2),
      drivetrainSubsystem.followPathCommand(true, "1BallAndStealPart1"),
      robotContainer.shootCommandHelper(),
      new ParallelCommandGroup(
        drivetrainSubsystem.followPathCommand(false, "1BallAndStealPart2"),
        new SequentialCommandGroup(
          new WaitCommand(0.25), 
          new CollectBalls(controller, collectorSubsystem, storageSubsystem, 1)
        )
      ),
      new RunStorageIn(storageSubsystem),
      new WaitCommand(.5),
      new StopStorage(storageSubsystem),
      drivetrainSubsystem.followPathCommand(false, "1BallAndStealPart3"),
      new EjectOneBallBottom(storageSubsystem, collectorSubsystem),
      drivetrainSubsystem.followPathCommand(false, "1BallAndStealPart4")
    );
  }
}
