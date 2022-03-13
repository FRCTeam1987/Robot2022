// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.oi;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RumbleWhile extends CommandBase {

  private final XboxController m_controller;
  private final BooleanSupplier m_shouldRumble;

  /** Creates a new RumbleWhile. */
  public RumbleWhile(final XboxController controller, final BooleanSupplier shouldRumble) {
    m_controller = controller;
    m_shouldRumble = shouldRumble;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double rumbleValue = m_shouldRumble.getAsBoolean() ? 1 : 0;
    m_controller.setRumble(RumbleType.kLeftRumble, rumbleValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_controller.setRumble(RumbleType.kLeftRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
