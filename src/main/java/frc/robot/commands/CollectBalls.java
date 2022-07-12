// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.collector.DeployCollector;
import frc.robot.commands.oi.RumbleWhile;
import frc.robot.commands.storage.RunStorageIn;
import frc.robot.commands.storage.WaitUntilBallsStored;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectBalls extends SequentialCommandGroup {

  private final CollectorSubsystem m_collector;
  private final StorageSubsystem m_storage;
  private final XboxController m_controller;

  /** Creates a new CollectMaxBalls. */
  public CollectBalls(final XboxController controller, final CollectorSubsystem collector, final StorageSubsystem storage, final int desiredBallCount) {
    // TODO don't run during auto
    m_collector = collector;
    m_storage = storage;
    m_controller = controller;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeployCollector(m_collector),
      new RunStorageIn(m_storage),
      new ParallelRaceGroup(
        new WaitUntilBallsStored(m_storage, desiredBallCount),
        new RumbleWhile(controller, () -> m_storage.isBallAtEntrance())
      ),
      new WaitCommand(0.2)
    );
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      m_collector.stow();
      m_collector.stop();
      m_storage.stop();

      if (DriverStation.isAutonomousEnabled() == false) {
        m_controller.setRumble(RumbleType.kLeftRumble, 0);
        m_controller.setRumble(RumbleType.kRightRumble, 0);
      }
  }
}
