// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util;
import frc.robot.subsystems.ShooterSubsystem;

public class SetHoodPosition extends CommandBase {

  private final ShooterSubsystem m_shooter;
  private final DoubleSupplier m_position;

  /** Creates a new SetHoodPosition. */
  public SetHoodPosition(final ShooterSubsystem shooter, final DoubleSupplier position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_position = position;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setHoodPosition(m_position.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.updateHoodPosition();
    if (interrupted) {
      DriverStation.reportWarning("Interrupted Command: Hood may not have reached the desired set point!", false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_shooter.updateHoodPosition();
    return m_shooter.isHoodAtDesiredPosition();
  }
}
