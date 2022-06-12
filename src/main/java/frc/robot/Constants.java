// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.InterpolatingDouble;
import frc.robot.lib.InterpolatingTreeMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double FALCON_ENCODER_RESOLUTION = 2048.0;
  public static final double FALCON_MAX_RPM = 6380.0;
  public static final int DRIVER_CONTROLLER_PORT = 0;

  public static final double DRIVETRAIN_PX_CONTROLLER = 0.3; // TODO tune this
  public static final double DRIVETRAIN_PY_CONTROLLER = DRIVETRAIN_PX_CONTROLLER;
  public static final double DRIVETRAIN_PTHETA_CONTROLLER = 6; // TODO tune this

  public static final String CANIVORE_CAN_BUS = "canfd";

  public static class Collector {
    public static final double COMMAND_DEPLOY_DURATION = 0.25;
    public static final double COMMAND_STOW_DURATION = 0.1;
    public static final int MOTOR_PDH_ID = 13;
    public static final int MOTOR_CAN_ID = 9;
    public static final double RUN_IN_SPEED = -1;
    public static final double RUN_OUT_SPEED = 0.5;
    public static final int SOLENOID_FORWARD_ID = 0;
    public static final int SOLENOID_REVERSE_ID = 1;
  }

  public static class Drivetrain {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.75);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.75);
    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = FALCON_MAX_RPM / 60.0 *
        SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI; //4.97
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_SQUARED = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        / 3.0;

    public static class TranslationGains {
      public static final double kP = 2.2956;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kA = 0.12872;
      public static final double kV = 2.3014;
      public static final double kS = 0.55493;
    }
    public static final SimpleMotorFeedforward TRANSLATION_FEED_FORWARD = new SimpleMotorFeedforward(
      TranslationGains.kS,
      TranslationGains.kV,
      TranslationGains.kA
    );

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(229.48); //-Math.toRadians(230.273);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(189.32); //-Math.toRadians(51.416);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 1;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 1;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(346.73); //-Math.toRadians(347.168);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(192.30); //-Math.toRadians(192.656);
  }

  public static class Shooter {
    public static final int LINEAR_ACTUATOR_LEFT_ID = 0;
    public static final int LINEAR_ACTUATOR_RIGHT_ID = 1;
    public static final int MOTOR_LEADER_PDH_ID = 4;
    public static final int MOTOR_LEADER_CAN_ID = 12;
    public static final int MOTOR_FOLLOWER_PDH_ID = 14;
    public static final int MOTOR_FOLLOWER_CAN_ID = 10;

    public static final double PRE_SHOOT_RPM = 2500;
    public static final double Offset_RPM_Increment_Amount = 50;
    public static final double Offset_RPM_Initial_Amount = 80; // 50 at heartland
    public static final double Shooter_RPM_Tolerance = 50;
    public static final double SHOOTER_REDUCTION = 1.0; //20.0/16.0;

    public static final int SOLENOID_FORWARD_ID = 5;
    public static final int SOLENOID_REVERSE_ID = 4;

    public static final double HubXPosition = 8.25;
    public static final double HubYPosition = 4.15;
  }

  public static class LimeLight {
    public final static double pipeline = 8;
    public final static double tyConfigLowThreshold = 3.25;
  }

    public static final class Targeting {

      public static class TreeMapValues{
        public static class Close {
            public final static double ty1 = 14.39; //TODO insert correect ty values.
            public final static double ty2 = -15.65; //TODO insert correect ty values.

            public final static double rpm1 = 2500; //TODO insert correect rpm values.
            public final static double rpm2 = 3075; //TODO insert correect rpm values.

            public final static double hood1 = 35; //TODO insert correect hood Heights mm values.
            public final static double hood2 = 70; //TODO insert correect hood Heights mm values.
        }
    }
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToShooter = new InterpolatingTreeMap<>();
    static {

      kDistanceToShooter.put(new InterpolatingDouble(16.06), new InterpolatingDouble(1800.0));  
      kDistanceToShooter.put(new InterpolatingDouble(14.0), new InterpolatingDouble(1800.0)); 
      kDistanceToShooter.put(new InterpolatingDouble(12.10), new InterpolatingDouble(1850.0)); 
       // hood lowered:
      // hood raised:
      kDistanceToShooter.put(new InterpolatingDouble(10.13), new InterpolatingDouble(1905.0));  
      kDistanceToShooter.put(new InterpolatingDouble(7.97), new InterpolatingDouble(1950.0));  
      kDistanceToShooter.put(new InterpolatingDouble(6.0), new InterpolatingDouble(1980.0));                                                                                                  
      kDistanceToShooter.put(new InterpolatingDouble(3.55), new InterpolatingDouble(2000.0));
      kDistanceToShooter.put(new InterpolatingDouble(1.01), new InterpolatingDouble(2050.0));
      kDistanceToShooter.put(new InterpolatingDouble(-1.02), new InterpolatingDouble(2085.0));
      kDistanceToShooter.put(new InterpolatingDouble(-1.4), new InterpolatingDouble(2125.0)); //Auto shot point (approximate)
      kDistanceToShooter.put(new InterpolatingDouble(-2.4), new InterpolatingDouble(2175.0));
      kDistanceToShooter.put(new InterpolatingDouble(-3.3), new InterpolatingDouble(2225.0));
      kDistanceToShooter.put(new InterpolatingDouble(-4.3), new InterpolatingDouble(2245.0));
      kDistanceToShooter.put(new InterpolatingDouble(-6.0), new InterpolatingDouble(2260.0));
      kDistanceToShooter.put(new InterpolatingDouble(-7.03), new InterpolatingDouble(2335.0));
      kDistanceToShooter.put(new InterpolatingDouble(-8.0), new InterpolatingDouble(2375.0));
      kDistanceToShooter.put(new InterpolatingDouble(-8.59), new InterpolatingDouble(2400.0));
      kDistanceToShooter.put(new InterpolatingDouble(-9.4), new InterpolatingDouble(2450.0));
      kDistanceToShooter.put(new InterpolatingDouble(-11.0), new InterpolatingDouble(2550.0));
      kDistanceToShooter.put(new InterpolatingDouble(-13.0), new InterpolatingDouble(2750.0));
      kDistanceToShooter.put(new InterpolatingDouble(-14.0), new InterpolatingDouble(2750.0));  
      kDistanceToShooter.put(new InterpolatingDouble(-15.0), new InterpolatingDouble(2920.0));  
      kDistanceToShooter.put(new InterpolatingDouble(-15.5), new InterpolatingDouble(3075.0));  

  }
    
    }

  public static class Storage {
    public static final double DEBOUNCE_DURATION_BOTTOM_LB = 0.075;
    public static final double DEBOUNCE_DURATION_TOP_LB = 0.04;
    public static final double DEBOUNCE_DURATION_BOTTOM_C = 0.04; //TODO Magic Number
    public static final double DEBOUNCE_DURATION_TOP_C = 0.05; //TODO Magic Number
    public static final int COLOR_SENSOR_TOP_ID = 2; //TODO Predicted IDs before sensors plugged in, verify accurate
    public static final int COLOR_SENSOR_BOTTOM_ID = 3; //TODO Predicted IDs before sensors plugged in, verify accurate
    public static final int DIGITAL_INPUT_BOTTOM_ID = 0;
    public static final int DIGITAL_INPUT_TOP_ID = 1;
    public static final int INITIAL_BALL_COUNT = 1;
    public static final int MAX_BALL_COUNT = 2;
    public static final int MOTOR_BOTTOM_CAN_ID = 8;
    public static final int MOTOR_TOP_CAN_ID = 9;
    public static final double RUN_IN_SPEED_BOTTOM = -0.75;
    public static final double RUN_IN_SPEED_TOP = -0.95;
    public static final double RUN_OUT_SPEED_BOTTOM = 0.95;
    public static final int SENSOR_BOTTOM = 9;
    public static final int SENSOR_TOP = 8;
    public static final double[] DEFAULT_COLOR = new double[]{0.0, 0.0, 0.0, 0.0};
    public static final double[] RED = new double[]{16500.0, 10300.0, 4800.0, 325.0};
    public static final double[] BLUE = new double[]{4900.0, 13300.0, 16300, 325.0};
    public static final double[] COLOR_TOLERANCE = new double[]{5000.0, 5000.0, 5000.0, 500.0};
    public static final double MAX_COLOR_SENSOR_PROXIMITY = 1750;

  
  }

  public static class Climber {
    public static final int Back_CLIMBER_MOTOR = 13;
    public static final int Front_CLIMBER_MOTOR = 14;
    // public static final int CLIMBER_PANCAKE_ONE = 4;
    // public static final int CLIMBER_PANCAKE_TWO = 5;
    // public static final int CLIMBER_PISTON_ONE = 2;
    // public static final int CLIMBER_PISTON_TWO = 3;
    public static final double CLIMBER_LOWER_TOLERANCE = 1;
    public static final double CLIMBER_SPEED = 0.35;
    public static final double CLIMBER_MAX_EXTEND_ANGLE = -6.5; /* magic number */
    public static final double initialExtendPosition = 20;
  }

  
}


