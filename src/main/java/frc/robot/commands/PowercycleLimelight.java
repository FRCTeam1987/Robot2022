// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PowercycleLimelight extends SequentialCommandGroup {

  private final PowerDistribution m_powerDistribution;
  /** Creates a new PowercycleLimelight. */
  public PowercycleLimelight(PowerDistribution powerDistribution) {
     m_powerDistribution = powerDistribution;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToggleLimelightOff(m_powerDistribution),
      new WaitCommand(.2),
      new ToggleLimelightOn(m_powerDistribution)
    );
  }

  @Override
  public boolean runsWhenDisabled() {
      // TODO Auto-generated method stub
      return true;
  }
}
