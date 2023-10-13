// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;

public class ArmAngleExtensionControl extends CommandBase {
  private final ExtensionSubsystem ExtensionSub;
  private final ArmSubsystem ArmSub;
  private boolean extended;
  private double target;
  private double p;
  /** Creates a new ArmAngleExtensionControl. */
  public ArmAngleExtensionControl(ExtensionSubsystem s_ExtensionSubsystem, ArmSubsystem s_ArmSubsystem, double target, double p, boolean extended) {
    ExtensionSub = s_ExtensionSubsystem;
    ArmSub = s_ArmSubsystem;
    this.extended = extended;
    this.p = p;
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ExtensionSubsystem, s_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //If we are supposed to extend and the forward mag limit negative(not yet reached)
    if (extended && !ExtensionSub.getForwardLimit()){
      ExtensionSub.safeSetArmExtensionMotor(0.50);
    }
    //If we are supposed to be retracted and the rear limit is not true
    if (!extended && !ExtensionSub.getRearLimit()){
      ExtensionSub.safeSetArmExtensionMotor(-0.50);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSub.setArmRotateMotor(0);
    ExtensionSub.setArmExtensionMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
