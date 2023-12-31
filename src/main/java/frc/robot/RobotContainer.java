// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ExtensionConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmAngleExtensionControl;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ArmControlPosition;
import frc.robot.commands.ExtensionControl;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.AutoChooser;
import frc.robot.commands.autos.AutoTrajectories;
import frc.robot.commands.autos.eventMap;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //Controllers
  private final CommandXboxController io_drivercontroller = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController io_opercontroller = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  //Subsystems
  private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final ArmSubsystem s_ArmSubsystem = new ArmSubsystem();
  private final IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();
  private final ExtensionSubsystem s_ExtensionSubsystem = new ExtensionSubsystem();
  private final eventMap map = new eventMap(s_Swerve, s_IntakeSubsystem, s_ArmSubsystem, s_ExtensionSubsystem);
  private final AutoTrajectories trajectories = new AutoTrajectories();
  private final AutoChooser chooser = new AutoChooser(trajectories, map.getMap(), s_Swerve);

  //Commands
  //Arm
  private final Command z_ExtendArm = new ExtensionControl(s_ExtensionSubsystem, ExtensionConstants.kArmExtensionSpeed);
  private final Command z_RetractArm = new ExtensionControl(s_ExtensionSubsystem, -ExtensionConstants.kArmExtensionSpeed);
  //Arm Spots
  private final Command z_HighScoreSpot = new ArmControlPosition(s_ArmSubsystem, 0.842, 5);
  //Arm extension and go to spots
  private final Command z_testExtendSpot = new ArmAngleExtensionControl(s_ExtensionSubsystem, s_ArmSubsystem, 0.842, 5, true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_ExtensionSubsystem.setDefaultCommand(new ExtensionControl(s_ExtensionSubsystem, () -> deadZone(-io_opercontroller.getLeftY(), OperatorConstants.kOperatorControllerDeadZone)));
    s_ArmSubsystem.setDefaultCommand(new ArmControl(s_ArmSubsystem, () -> deadZone(io_opercontroller.getRightY(), OperatorConstants.kOperatorControllerDeadZone)));
    s_IntakeSubsystem.setDefaultCommand(new IntakeControl(s_IntakeSubsystem, () -> io_opercontroller.getRightTriggerAxis(), () -> io_opercontroller.getLeftTriggerAxis()));
    //Drive
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
          s_Swerve, 
          () -> deadZone(-io_drivercontroller.getLeftY(), OperatorConstants.kDriveControllerDeadZone), 
          () -> deadZone(-io_drivercontroller.getLeftX(), OperatorConstants.kDriveControllerDeadZone), 
          () -> deadZone(-io_drivercontroller.getRightX(), OperatorConstants.kDriveControllerDeadZone), 
          () -> false //Robot will always be field centric
        )
    );

    CameraServer.startAutomaticCapture();
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData("Auto Choices", chooser.getAutoChooser());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    io_opercontroller.rightBumper().whileTrue(z_ExtendArm);
    io_opercontroller.leftBumper().whileTrue(z_RetractArm);
    io_opercontroller.a().whileTrue(z_HighScoreSpot);
    io_opercontroller.b().whileTrue(z_testExtendSpot);
  }

  public static double deadZone(double rawInput, double deadband){
    if (rawInput > deadband){
      return rawInput;
    }
    if (rawInput < -deadband){
      return rawInput;
    }
    return 0; //We are within the deadband. Return 0.
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    s_Swerve.zeroGyro();
    return chooser.getCommand();
  }
}
