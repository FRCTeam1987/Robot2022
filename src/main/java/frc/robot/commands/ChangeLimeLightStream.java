// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChangeLimeLightStream extends InstantCommand {

  public static enum StreamType {
    standard,
    pipMain,
    pipSecondary
  }

  private final StreamType m_streamType;

  public ChangeLimeLightStream(final StreamType streamType) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_streamType = streamType;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(m_streamType.ordinal());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
