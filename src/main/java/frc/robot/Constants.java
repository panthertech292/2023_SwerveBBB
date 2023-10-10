// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Constants for our controllers and operation of the robot
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveControllerDeadZone = 0.10;
    public static final double kOperatorControllerDeadZone = 0.15; //These are different, because we use cheaper controllers for our operator.
  }

  //Constants for the arm mechanism of the robot (rotation and extension)
  public static final class ArmConstants {
    //Constants for the physical configuration of the hardware.
    //Motors
    public static final int kArmRotateMotorID = 20;
    public static final int kArmExtensionMotorID = 40; //TODO: This is a placeholder!
    //Sensors
    public static final int kArmRotateEncoderID = 41; //TODO: This is a placeholder!
    public static final int kRearMagneticLimitID = 0; //TODO: This is a placeholder!
    public static final int kForwardMagneticLimitID = 1; //TODO: This is a placeholder!

    public static final double kMinArmTheta = 0; //TODO: This is a placeholder!
    public static final double kMaxArmTheta = 360; //TODO: This is a placeholder!
  }
  //Constants for the intake of the robot
  public static final class IntakeConstants {
    //Hardware Constants
    public static final int kIntakeMotorUpID = 50; //TODO: This is a placeholder!
    public static final int kIntakeMotorDownID = 51; //TODO: This is a placeholder!
  }
}