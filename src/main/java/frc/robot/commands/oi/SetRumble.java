// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetRumble extends InstantCommand {

  public static enum RumbleValue {
    Off,
    On
  }

  private final XboxController m_controller;
  private final RumbleValue m_rumbleValue;

  public SetRumble(final XboxController controller, final RumbleValue rumbleValue) {
    m_controller = controller;
    m_rumbleValue = rumbleValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setRumble(RumbleType.kLeftRumble, m_rumbleValue.ordinal());
    m_controller.setRumble(RumbleType.kRightRumble, m_rumbleValue.ordinal());
  }
}
