// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeControl extends CommandBase {
  private final IntakeSubsystem IntakeSub;
  private DoubleSupplier intakeSpeed;
  /** Creates a new IntakeRun. */
  public IntakeControl(IntakeSubsystem s_IntakeSubsystem, DoubleSupplier speed) {
    IntakeSub = s_IntakeSubsystem;
    intakeSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSub.setIntake(intakeSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSub.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
