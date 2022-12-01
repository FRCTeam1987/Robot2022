// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class BrakeClimber extends InstantCommand {
  
  private final TelescopeSubsystem m_telescopeFront;
  private final TelescopeSubsystem m_telescopeBack;

  public BrakeClimber(TelescopeSubsystem telescopeFront, TelescopeSubsystem telescopeBack) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_telescopeFront = telescopeFront;
    m_telescopeBack = telescopeBack;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_telescopeFront.brakeMotor();
    m_telescopeBack.brakeMotor();
  }

  @Override
  public boolean runsWhenDisabled() {
      // TODO Auto-generated method stub
      return true;
  }
}
