package frc.robot;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

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

    static boolean isLimelightAiming = false;

    static String status = "ready";


    static Queue<String> trajectoryQueue;
    static PathPlannerTrajectory currentTrajectory;// = PathPlanner.loadPath("New Path", 3, 2.5);
    static double kP = 3;

    static int mirrorXaxis = 1;
    
    static HolonomicDriveController driveController = new HolonomicDriveController(new PIDController(kP, 0, 0),
            new PIDController(kP, 0, 0),
            new ProfiledPIDController(kP * SwerveDef.MAX_SPEED_RADPS / SwerveDef.MAX_SPEED_MPS, 0, 0,
                    new Constraints(SwerveDef.MAX_SPEED_RADPS, SwerveDef.MAX_SPEED_RADPS)));

    
    public static void init() {
        int position = createSmartDashboardNumber("position", -1);
        int team = createSmartDashboardNumber("team", -1);

        if(position != -1 && team != -1) {
            mirrorXaxis = team * 2 - 1;
            loadTrajectory("first" + Integer.toString(position));
        }

        Rameno.startRelease();
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
                    Robot.SWERVE.drive(speeds.vxMetersPerSecond * mirrorXaxis, speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond);
                    runCurrentEvents(trajectory.getMarkers(), lastElapsedTime,elapsedTime);
                } else {
                    Robot.SWERVE.drive(0, 0, 0);
                    Robot.SWERVE.holdAngle = ((PathPlannerState) trajectory.getEndState()).holonomicRotation
                            .getRadians();
                    status = "ready";
                }
                break;
            case "ready":
                if(!trajectoryQueue.isEmpty()){
                    loadTrajectory(trajectoryQueue.remove());
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

            case "release_rameno": //does in the start
                Rameno.startRelease();
                break;

            case "set_rameno_0":
                Rameno.changePositionAutonomous(0);
                break;
            case "set_rameno_1":
                Rameno.changePositionAutonomous(1);
                break;
            case "set_rameno_2":
                Rameno.changePositionAutonomous(2);
                break;
            case "set_rameno_3":
                Rameno.changePositionAutonomous(3);
                break;
            case "set_rameno_4":
                Rameno.changePositionAutonomous(4);
                break;


            case "outtake_gripper_box":
                Gripper.gripAutonomous(false, false);
                break;
            case "intake_gripper_box":
                Gripper.gripAutonomous(true, false);
                break;
            case "outtake_gripper_cone":
                Gripper.gripAutonomous(false, true);
                break;
            case "intake_gripper_cone":
                Gripper.gripAutonomous(true, true);
                break;
            case "disable_gripper":
                Gripper.disableMotor();

            default:
                System.out.println("Event " + eventName + " was not found.");
                break;
        }
    } 

    public static int createSmartDashboardNumber(String key, double defValue) {

        // See if already on dashboard, and if so, fetch current value
        int value = (int)SmartDashboard.getNumber(key, defValue);
      
        // Make sure value is on dashboard, puts back current value if already set
        // otherwise puts back default value
        SmartDashboard.putNumber(key, value);
      
        return value;
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