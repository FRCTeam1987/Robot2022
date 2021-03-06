// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleLimelightOn extends InstantCommand {

  private final PowerDistribution m_powerDistribution;

  public ToggleLimelightOn(PowerDistribution powerDistribution) {
    // Use addRequirements() here to declare subsystem dependencies.
      m_powerDistribution = powerDistribution;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_powerDistribution.setSwitchableChannel(true);
  }

  @Override
  public boolean runsWhenDisabled() {
    // TODO Auto-generated method stub
    return true;
  }
}
