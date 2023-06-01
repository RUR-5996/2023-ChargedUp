package frc.robot;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

    static Timer startTimer = new Timer();


    static Queue<String> trajectoryQueue;
    static PathPlannerTrajectory currentTrajectory;// = PathPlanner.loadPath("New Path", 3, 2.5);
    static double kP = 2;

    static int mirrorXaxis = 1;
    
    static HolonomicDriveController driveController = new HolonomicDriveController(new PIDController(kP, 0, 0),
            new PIDController(kP, 0, 0),
            new ProfiledPIDController(kP * SwerveDef.MAX_SPEED_RADPS / SwerveDef.MAX_SPEED_MPS, 0, 0,
                    new Constraints(SwerveDef.MAX_SPEED_RADPS, SwerveDef.MAX_SPEED_RADPS)));

    public static void robotInit(){
        createSmartDashboardNumber("position", -1);
        createSmartDashboardNumber("team", -1);
    }

    public static void init() {
        int position = createSmartDashboardNumber("position", -1);
        int team = createSmartDashboardNumber("team", -1);
        
        SwerveDef.flDrive.setNeutralMode(NeutralMode.Brake);
        SwerveDef.frDrive.setNeutralMode(NeutralMode.Brake);
        SwerveDef.rlDrive.setNeutralMode(NeutralMode.Brake);
        SwerveDef.rrDrive.setNeutralMode(NeutralMode.Brake);

        //temporary
        position = 7;
        team = 1;

        trackLine = new TrackLine()
            .Add(new TrackPoint(0, 0, 0))
            .Add(new TrackPoint(Math.PI, 0,0 ));

        Rameno.startRelease();
        startTimer.reset();
        startTimer.start();
        waiting = false;

        if(position != -1 && team != -1) {
            mirrorXaxis = team * 2 - 1;
            loadTrajectory("first" + Integer.toString(position));
        }

        Robot.startTime = Timer.getFPGATimestamp();
    }
    
    static TrackLine trackLine;
    static boolean waiting = false;
    static double waitFinishTime = 0;
    
    public static void periodic(){
        
        TargettingVelocity targetVelocity = trackLine.Get(Robot.SWERVE.odometry.getPoseMeters());
        if(startTimer.get() < 1){
            Robot.SWERVE.drive(targetVelocity.xSpeed, targetVelocity.ySpeed, targetVelocity.rotation);
            //runTrajectory(currentTrajectory, Robot.SWERVE.odometry);
        }
        else{
           Robot.SWERVE.drive(0, 0, 0); 
        }

        

        
        
    }
    public static void loadTrajectory(String pathName){ 
        currentTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(2, 3)); //double check with info in path planner and see if this has any influence on robot speed
        newTrajectory();
    }
    public static boolean isReady(){
        return status.equals("ready");
    }

    static void newTrajectory() {
        status = "setup";
    }
    static final double SPEED_COEF = 3.59/9.69;

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
                showEvents(trajectory.getMarkers());
                break;

            case "execute":
                if (elapsedTime < (trajectory.getEndState()).timeSeconds) {
                    ChassisSpeeds speeds = driveController.calculate(odometry.getPoseMeters(),
                            ((PathPlannerState) trajectory.sample(elapsedTime)),
                            ((PathPlannerState) trajectory.sample(elapsedTime)).holonomicRotation);
                    Robot.SWERVE.drive(-speeds.vxMetersPerSecond*/*mirrorXaxis**/SPEED_COEF, -speeds.vyMetersPerSecond*SPEED_COEF,
                            -speeds.omegaRadiansPerSecond); //prohodit vx a vy???
                    SmartDashboard.putNumber("rotation auto", speeds.omegaRadiansPerSecond);
                    SmartDashboard.putNumber("vx auto", speeds.vxMetersPerSecond);
                    SmartDashboard.putNumber("vy auto", -speeds.vyMetersPerSecond);
                    runCurrentEvents(trajectory.getMarkers(), lastElapsedTime,elapsedTime);
                } else {
                    Robot.SWERVE.drive(0, 0, 0);
                    Robot.SWERVE.holdAngle = ((PathPlannerState) trajectory.getEndState()).holonomicRotation
                            .getRadians();
                    status = "stop";
                    
                }
                break;
            case "stop":
                SwerveDef.rlModule.setAngle(135);
                SwerveDef.rrModule.setAngle(45);
                SwerveDef.frModule.setAngle(135);
                SwerveDef.flModule.setAngle(45);
                status = "ready";
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

    static void showEvents(List<EventMarker> allMarkers){
        for(EventMarker eventMarker : allMarkers){
            int i = 0;
            for(String eventName : eventMarker.names){
                SmartDashboard.putString(eventName,Double.toString(eventMarker.timeSeconds) + " : " + Integer.toString(i));
                i+=1;
            }
        }
    }


    static void runCurrentEvents(List<EventMarker> allMarkers, double timeFrom, double timeTo){
        SmartDashboard.putNumber("auto time from",timeFrom);
        SmartDashboard.putNumber("auto time to",timeTo);
       for(EventMarker eventMarker : allMarkers){
            if(eventMarker.timeSeconds >= timeFrom && eventMarker.timeSeconds < timeTo){
                runEvents(eventMarker.names);
            }
        }
    }
    static void runEvents(List<String> eventNames){
        for(String eventName : eventNames){
            SmartDashboard.putString("das", eventName);
            runEvent(eventName);
        }
    }
    static void runEvent(String eventName){
        switch(eventName){
            case "print_test":
                System.out.println("Test event called.");
                break;
            case "print_trajectory_time":
                SmartDashboard.putNumber("elapsed time", elapsedTime);
                break;

            case "release_rameno": //does in the start
                Rameno.startRelease();
                break;
            case "engage_rameno":
                Rameno.engaged = true;
                break;
            case "disengage_rameno":
                Rameno.engaged = false;
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
            case "set_rameno_5":
                Rameno.changePositionAutonomous(5);
                break;

            case "wait_1":
                waitFinishTime = startTimer.get()+1;
                startTime += 1;
                waiting = true;
                break;

            case "wait_3":
                waitFinishTime = startTimer.get()+3;
                startTime += 3;
                waiting = true;
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

// class that describes one state in which the robot should be, recognized by the odomtetry
class TrackPoint
{
    public double rotationRadians;
    public double positionX;
    public double positionY;
    public String[] commands;

    

    public TrackPoint(double rotRad,double x, double y){
        positionX = x;
        positionY = y;
        rotationRadians = TrackLine.FormatRotation(rotRad);
        
        
    }
}

// used to pass the speed values directly to the move method (look TrackLine.Get())
class TargettingVelocity{
    public double rotation;
    public double xSpeed;
    public double ySpeed;

    public static final TargettingVelocity Empty = new TargettingVelocity(0, 0, 0);

    public TargettingVelocity(double rot, double x, double y){
        rotation = rot;
        xSpeed = x;
        ySpeed = y;
    }
}

// list of trackpoints that determines the robots movement and functions in autonomous
class TrackLine{
    private ArrayList<TrackPoint> points;
    private int index;
    private boolean finished;


    public TrackLine(){
        points = new ArrayList<TrackPoint>();
        index = 0;
        finished = false;
    } 

    public boolean IsFinished(){
        return finished;
    }

    public TrackLine Restart(){
        index = 0;
        finished = false;
        return this;
    }

    public TrackLine Add(TrackPoint point){
        points.add(point);
        return this;
    }

    public static final double MaxDeflectionPosition = 0.2;
    public static final double MaxDeflectionRotation = 0.1;

    // returns the values by such the robot should move
    public TargettingVelocity Get(Pose2d gyroPos){
        

        TargettingVelocity targetVelocity = new TargettingVelocity(0,0,0);

        DetectThere(gyroPos.getX(),gyroPos.getY(), gyroPos.getRotation().getRadians());
        
        // TO DO

        if(finished) return targetVelocity;
        
        // TO DO

        TrackPoint targetPoint = points.get(index);
        targetVelocity.xSpeed = gyroPos.getX();
        targetVelocity.ySpeed = gyroPos.getY();
        targetVelocity.rotation = gyroPos.getRotation().getRadians();
        
        return targetVelocity;

    }

    public static double FormatRotation(double rotRad){
        while(rotRad < 0){
            rotRad += 2 * Math.PI;
        }
        while(rotRad > 2 * Math.PI){
            rotRad += 2 * Math.PI;
        }
        return rotRad;
    }

    public boolean DetectThere(double posX, double posY, double rotRad){
        TrackPoint targetPoint = points.get(index);
        if(Math.abs(targetPoint.positionX - posX) < MaxDeflectionPosition){
            if(Math.abs(targetPoint.positionY - posY) < MaxDeflectionPosition){
                if(FormatRotation(targetPoint.rotationRadians - FormatRotation(rotRad)) < MaxDeflectionRotation){
                    index++;
                    return true;
                }
            }
        }
        
        return false; 
    }

}