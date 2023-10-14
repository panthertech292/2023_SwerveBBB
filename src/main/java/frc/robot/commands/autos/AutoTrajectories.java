package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;

public class AutoTrajectories {

    private PathPlannerTrajectory simpleDriveForward;

    private final PathConstraints slowConstraints;//, constraints;

    public AutoTrajectories() {
        //constraints = new PathConstraints(Constants.SwerveConstants.maxSpeed, Constants.SwerveConstants.maxAccel);
        slowConstraints = new PathConstraints(Constants.AutoConstants.slowVel, Constants.AutoConstants.slowAccel);
    }

    public PathPlannerTrajectory simpleDriveForward() {
        simpleDriveForward = PathPlanner.loadPath("simpleDriveForward", slowConstraints);
        return simpleDriveForward;
    }

}