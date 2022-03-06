// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.storage.FeedShooter;
import frc.robot.commands.storage.StopStorage;
import frc.robot.commands.storage.WaitForBallShoot;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EjectOneBallTop extends SequentialCommandGroup {
  private final ShooterSubsystem m_shooter;
  private final StorageSubsystem m_storage;
  /** Creates a new EjectOneBallTop. */
  public EjectOneBallTop(final StorageSubsystem storage, final ShooterSubsystem shooter) {
    m_shooter = shooter;
    m_storage = storage;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterRpm(m_shooter, () -> 1000),
      new FeedShooter(m_storage),
      new WaitForBallShoot(m_storage, 1),
      new WaitCommand(0.25),
      new StopStorage(m_storage),
      new SetShooterRpm(m_shooter, () -> 0)
    );
  }

  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    if (interrupted) {
      m_shooter.stop();
      m_storage.stop();
      m_storage.decrementBallCount();
    }
  }
}
