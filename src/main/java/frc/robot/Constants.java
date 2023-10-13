// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

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
    //Sensors
    public static final int kArmRotateEncoderID = 9;
    //Values
    public static final double kMinArmTheta = 0; //TODO: This is a placeholder!
    public static final double kMaxArmTheta = 360; //TODO: This is a placeholder!
  }
  //Constants for extending and retracting the arm
  public static final class ExtensionConstants {
    public static final int kArmExtensionMotorID = 30;
    public static final int kRearMagneticLimitID = 8;
    public static final int kForwardMagneticLimitID = 1;
    //Values
    public static final double kArmExtensionSpeed = 0.20;
  }
  //Constants for the intake of the robot
  public static final class IntakeConstants {
    //Hardware Constants
    public static final int kIntakeMotorUpID = 40;
    public static final int kIntakeMotorDownID = 41;
  }
  public static final class SwerveConstants{
    //Hardware Setup
    public static final int kPigeon2ID = 9;
    //Front Left Module 0
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(277.12);
    }
    //Front Right Module 1
    public static final class Mod1 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(223.7); //Might be drifting?
    }
    //Back Left Module 2
    public static final class Mod2 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180.7);
    }
    //Back Right Module 3
    public static final class Mod3 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 14;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(162.4);
    }
    
  }
}
