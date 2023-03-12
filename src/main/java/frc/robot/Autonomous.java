package frc.robot;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Autonomous {
    

    static double startTime = 0;
    static double elapsedTime;
    static double lastElapsedTime;

    static String status = "ready";

    static PathPlannerTrajectory currentTrajectory;// = PathPlanner.loadPath("New Path", 3, 2.5);
    static double kP = 3;
    
    static HolonomicDriveController driveController = new HolonomicDriveController(new PIDController(kP, 0, 0),
            new PIDController(kP, 0, 0),
            new ProfiledPIDController(kP * SwerveDef.MAX_SPEED_RADPS / SwerveDef.MAX_SPEED_MPS, 0, 0,
                    new Constraints(SwerveDef.MAX_SPEED_RADPS, SwerveDef.MAX_SPEED_RADPS)));

    
    public static void init() {
        loadTrajectory("test1");
    }

    public static void periodic(){
        runTrajectory(currentTrajectory, Robot.SWERVE.odometry);
    }
    public static void loadTrajectory(String pathName){
        currentTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(4, 3)); 
        newTrajectory();
    }
    public static boolean isReady(){
        return status.equals("ready");
    }

    static void newTrajectory() {
        status = "setup";
    }

    static void runTrajectory(PathPlannerTrajectory trajectory, SwerveDriveOdometry odometry/*, Rotation2d rot2d*/) {
        lastElapsedTime = elapsedTime;
        elapsedTime = Timer.getFPGATimestamp() - startTime;

        switch (status) {
            case "setup":
                Robot.SWERVE.setOdometry((trajectory.getInitialState()).poseMeters,
                        (trajectory.getInitialState()).poseMeters.getRotation());
                startTime = Timer.getFPGATimestamp();
                status = "execute";
                System.out.println("setup");
                break;

            case "execute":
                if (elapsedTime < (trajectory.getEndState()).timeSeconds) {
                    ChassisSpeeds speeds = driveController.calculate(odometry.getPoseMeters(),
                            ((PathPlannerState) trajectory.sample(elapsedTime)),
                            ((PathPlannerState) trajectory.sample(elapsedTime)).holonomicRotation);
                    Robot.SWERVE.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond);
                    runCurrentEvents(trajectory.getMarkers(), lastElapsedTime,elapsedTime);
                } else {
                    Robot.SWERVE.drive(0, 0, 0);
                    Robot.SWERVE.holdAngle = ((PathPlannerState) trajectory.getEndState()).holonomicRotation
                            .getRadians();
                    status = "ready";
                }
                break;
            default:
                Robot.SWERVE.drive(0, 0, 0);
                break;
        }
    }

    static void runCurrentEvents(List<EventMarker> allMarkers, double timeFrom, double timeTo){
       for(EventMarker eventMarker : allMarkers){
            if(eventMarker.timeSeconds >= timeFrom && eventMarker.timeSeconds < timeTo){
                runEvents(eventMarker.names);
            }
        }
    }
    static void runEvents(List<String> eventNames){
        for(String eventName : eventNames){
            runEvent(eventName);
        }
    }
    static void runEvent(String eventName){
        switch(eventName){
            case "print_test":
                System.out.println("Test event called.");
                break;
            case "print_trajectory_time":
                System.out.println(elapsedTime);
                break;
            case "aim_lower_stick":
                System.out.println("Targeting lower stick...");
                AimLowerStick();
                break;
            case "aim_ground":
                System.out.println("Targeting lower stick...");
                // TODO
                break;
            // case "" TODO
            default:
                System.out.println("Event " + eventName + " was not found.");
        }
    } 

    public static void AimLowerStick(){
        // TODO
    }   


    /*public static void init(){
        Events.put("test",new PrintCommand("Command test was executed."));
    }*/
    
   /*  static void runPath(PathPlannerTrajectory trajectory){
        FollowPathWithEvents command = new FollowPathWithEvents(
            getFollowPathCommand(currentPathGroup.get(0), false),
            currentPathGroup.get(0).getMarkers(),
            Events
        );       
        //command.execute();
    };
    static Command getFollowPathCommand(PathPlannerTrajectory traj, boolean isFirstPath){
        return new PrintCommand("not yet");
    } */

}