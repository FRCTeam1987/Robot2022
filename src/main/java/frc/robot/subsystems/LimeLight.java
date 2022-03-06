// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class LimeLight extends SubsystemBase {
    public static final String LIMELIGHT = "limelight";
  
    /** Creates a new LimeLight. */
    public LimeLight() {
      final ShuffleboardTab tab = Shuffleboard.getTab("LimeLight");
      final InstantCommand turnOnLEDsCommand = new InstantCommand(() -> {
        turnOnLEDs();
      });
      turnOnLEDsCommand.runsWhenDisabled();
      tab.add("Turn On LEDs", turnOnLEDsCommand);
      final InstantCommand turnOffLEDsCommand = new InstantCommand(() -> {
        turnOffLEDs();
      });
      turnOffLEDsCommand.runsWhenDisabled();
      tab.add("Turn Off LEDs", turnOffLEDsCommand);
      final NetworkTableEntry pipelineSelector = tab.add("Pipeline Selector", Constants.LimeLight.pipeline).getEntry();
      final InstantCommand setPipelineCommand = new InstantCommand(() -> {
        setPipeline((double) pipelineSelector.getNumber(Constants.LimeLight.pipeline));
      });
      setPipelineCommand.runsWhenDisabled();
      tab.add("Set Pipeline", setPipelineCommand);
      setPipeline(pipelineSelector.getDouble(Constants.LimeLight.pipeline));
    }


@Override
public void periodic() {
  // This method will be called once per scheduler run
}

public double getYAxis(){
  return NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("ty").getDouble(0);
}

public double getXAxis(){
  return NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("tx").getDouble(0);
}

public double getVisible(){
  return NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("tv").getDouble(0);
}

public boolean canSeeTarget() {
  return getVisible() == 1;
}

public void init() {
  setPipeline(Constants.LimeLight.pipeline);
  // turnOffLEDs();
}

public void setPipeline(final double pipeline) {
  NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("pipeline").setNumber(pipeline);
}

public void turnOnLEDs(){
  NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("ledMode").setNumber(3);
}

public void turnOffLEDs(){
  NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("ledMode").setNumber(1);
}
}