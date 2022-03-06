// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.collector.DeployCollector;
import frc.robot.commands.collector.StowCollector;
import frc.robot.commands.storage.StopStorage;
import frc.robot.commands.storage.WaitForBallShoot;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EjectOneBallBottom extends SequentialCommandGroup {
  private final CollectorSubsystem m_collector;
  private final StorageSubsystem m_storage;
  /** Creates a new EjectOneBallTop. */
  public EjectOneBallBottom(final StorageSubsystem storage, final CollectorSubsystem collector) {
    m_collector = collector;
    m_storage = storage;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new InstantCommand(() -> m_storage.runForIntake()).andThen(new WaitCommand(0.25)),
        new InstantCommand(),
        () -> m_storage.getBallCount() == 1
      ),
      new InstantCommand(() -> {
        m_collector.deploy();
        m_collector.runRollerOut();
        m_storage.runForOutput();
      }, m_collector),
      new WaitForBallShoot(m_storage, false, 1),
      new WaitCommand(0.125),
      new StopStorage(m_storage),
      new StowCollector(m_collector)
      // new SetShooterRpm(m_shooter, () -> 0)
    );
  }
  
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    if (interrupted) {
      m_storage.stop();
      m_storage.decrementBallCount();
      m_collector.stop();
      m_collector.stow();
    }
  }
}
