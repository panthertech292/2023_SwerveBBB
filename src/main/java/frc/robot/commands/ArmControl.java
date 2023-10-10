// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmControl extends CommandBase {
  private final ArmSubsystem ArmSub;
  private DoubleSupplier armSpeed;
  /** Creates a new ArmControl. */
  public ArmControl(ArmSubsystem s_ArmSubsystem, DoubleSupplier speed) {
    ArmSub = s_ArmSubsystem;
    armSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmSub.setArmRotateMotor(armSpeed.getAsDouble()); //TODO: Find out if we can use this with soft limits, or if we need to code our own.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSub.setArmRotateMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
