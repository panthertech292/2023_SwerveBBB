// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubsystem;

public class ExtensionControl extends CommandBase {
  private final ExtensionSubsystem ExtensionSub;
  private double speed;
  private DoubleSupplier speedSupplier;
  private Boolean doubleSupplierUsed;
  /** Creates a new ExtensionControl. */
  public ExtensionControl(ExtensionSubsystem s_ExtensionSubsystem, double speed) {
    ExtensionSub = s_ExtensionSubsystem;
    this.speed = speed;
    doubleSupplierUsed = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ExtensionSubsystem);
  }
  public ExtensionControl(ExtensionSubsystem s_ExtensionSubsystem, DoubleSupplier speed) {
    ExtensionSub = s_ExtensionSubsystem;
    this.speedSupplier = speed;
    doubleSupplierUsed = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ExtensionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!doubleSupplierUsed){
      ExtensionSub.safeSetArmExtensionMotor(speed);
    }
    if(doubleSupplierUsed){
      ExtensionSub.safeSetArmExtensionMotor(speedSupplier.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ExtensionSub.setArmExtensionMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
