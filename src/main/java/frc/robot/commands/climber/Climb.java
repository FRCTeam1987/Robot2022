// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Util;
import frc.robot.commands.drivetrain.WaitUntilRoll;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb extends SequentialCommandGroup {
  private final TelescopeSubsystem m_telescopeFront;
  private final TelescopeSubsystem m_telescopeBack;
  private final DrivetrainSubsystem m_drivetrain;
  private final Compressor m_compressor;
  private final XboxController m_xbox;
  /** Creates a new Climb. */
  public Climb(TelescopeSubsystem telescopeFront, TelescopeSubsystem telescopeBack, DrivetrainSubsystem drivetrain, final XboxController xbox, Compressor compressor) {

    m_telescopeFront = telescopeFront;
    m_telescopeBack = telescopeBack;
    m_drivetrain = drivetrain;
    m_xbox = xbox;
    m_compressor = compressor;

    addCommands(
      new ClimbStep1(m_telescopeFront, m_compressor),
      new WaitUntilCommand(() -> m_xbox.getAButtonPressed()),
      new ClimbStep2(m_telescopeFront, m_telescopeBack, m_drivetrain),
      new WaitUntilRoll(m_drivetrain, true, 45), // TODO find angle
      new ClimbStep3(m_telescopeFront, m_telescopeBack, m_drivetrain),
      new WaitUntilRoll(m_drivetrain, true, 45), // TODO find angle
      new ClimbStep4(m_telescopeFront, m_telescopeBack, m_drivetrain)

      
    );
  }
}
