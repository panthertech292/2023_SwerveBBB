package frc.robot.commands.autos;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmControlPosition;
import frc.robot.commands.ExtensionControl;
import frc.robot.commands.IntakeControl;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class eventMap {
    public HashMap<String, Command> eventMap = new HashMap<String, Command>();
    private final SwerveSubsystem s_SwerveSub;
    private final IntakeSubsystem s_IntakeSub;
    private final ArmSubsystem s_ArmSub;
    private final ExtensionSubsystem s_ExtensionSub;

    public eventMap(SwerveSubsystem s_SwerveSub, IntakeSubsystem s_IntakeSub, ArmSubsystem s_ArmSub, ExtensionSubsystem s_ExtensionSub) {
        this.s_SwerveSub = s_SwerveSub;
        this.s_IntakeSub = s_IntakeSub;
        this.s_ArmSub = s_ArmSub;
        this.s_ExtensionSub = s_ExtensionSub;
        //eventMap.put("balance", new InstantCommand(() -> s_Swerve.autoBalance())); //  was just autoBalance *** balance\ on charge station
        //eventMap.put("autoCorrect", new InstantCommand(() -> s_Swerve.rotateToDegree(180)));

        eventMap.put( // scores cube and stows automatically
            "scoreCubeLow", 
            Commands.sequence(
                new IntakeControl(s_IntakeSub, () -> 0, () -> 1)
            )
        );
        eventMap.put(
            "scoreCubeMid",
            Commands.sequence(
                new ArmControlPosition(s_ArmSub, 0.8, 5), //TODO: Get target for this.
                new WaitCommand(2),
                new IntakeControl(s_IntakeSub, () -> 0, () -> 1)
            )
        );
        eventMap.put(
            "scoreCubeHigh",
            Commands.sequence(
                new ExtensionControl(s_ExtensionSub, 0.30),
                new WaitCommand(2),
                new ArmControlPosition(s_ArmSub, 0.8, 5), //TODO: Get target for this.
                new WaitCommand(2),
                new IntakeControl(s_IntakeSub, () -> 0, () -> 1)
            )
        );
    }

    public HashMap<String, Command> getMap() {
        return eventMap;
    }

}
