package org.firstinspires.ftc.teamcode.autonomous;
//6.92307692308 is 1 inch
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class SUBMERSIBLE extends LinearOpMode {
    public static double DISTANCE = 24; // in

    public Slide slide;
    public Motors panningMotor;
    public Servos claw;
    public Servos panningServo;
    public Servos orientation;
    public Motors pulley;
    public HuskyLenses frontLens;

    public HuskyLens.Block firstBlock;
    int xCenter = 210;
    int xDifference = 0;
    double xMovement;
    double scale1 = 0.06;
    double scale2 = 0.041;

    public int runFrames;

    public HuskyLens.Block currentTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        slide = new Slide(hardwareMap, telemetry, "slide");
        panningMotor = new Motors(hardwareMap, "panningmotor");
        claw = new Servos(hardwareMap, "claw");
        panningServo = new Servos(hardwareMap, "panning");
        orientation = new Servos(hardwareMap, "orientation");
        pulley = new Motors(hardwareMap, "pulley");
//        frontLens = new HuskyLenses(hardwareMap, "frontlens", "color");

        runFrames = 0;
        claw.moveForwardMAX();
        panningServo.moveBackwardMIN();
        orientation.moveBackwardMIN();
        slide.MoveToLevel(Slide.level.zero);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(7, 60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory firstspec = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(3,34)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();


        Trajectory alligntopush1 = drive.trajectoryBuilder(firstspec.end())
                .lineTo(new Vector2d(36, 60)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();
        Trajectory alligntopush2 = drive.trajectoryBuilder(alligntopush1.end())
                .lineTo(new Vector2d(40, 17)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();
        Trajectory alligntopush3 = drive.trajectoryBuilder(alligntopush2.end())
                .lineTo(new Vector2d(55, 17)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();


        Pose2d pushPose = new Pose2d(0, 0, Math.toRadians(90));

        Trajectory pushfirst = drive.trajectoryBuilder(pushPose)
                .lineTo(new Vector2d(-10, 30)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();
        Trajectory backfirst = drive.trajectoryBuilder(pushfirst.end())
                .lineTo(new Vector2d(-15, 0)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();
        Trajectory flafe = drive.trajectoryBuilder(backfirst.end())
                .strafeTo(new Vector2d(10, 0)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();
        Trajectory pushsecond = drive.trajectoryBuilder(flafe.end())
                .lineTo(new Vector2d(8, 35))
                .build();

        Pose2d startcycling = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d startcycling2 = new Pose2d(-2, 0, Math.toRadians(90));
        Pose2d startcycling3 = new Pose2d(-2, 0, Math.toRadians(90));

        Trajectory toBara = drive.trajectoryBuilder(startcycling)
                .strafeTo(new Vector2d(-102, -60)
                ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();
        Trajectory toBarb = drive.trajectoryBuilder(startcycling2)
                .strafeTo(new Vector2d(-94, -60)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();
        Trajectory toBarc = drive.trajectoryBuilder(startcycling3)
                .strafeTo(new Vector2d(-90, -60)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();

        Pose2d comebackpose = new Pose2d(0, 0, Math.toRadians(90));

        Trajectory backtopickup = drive.trajectoryBuilder(comebackpose)
                .lineTo(new Vector2d(97, 62)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();

        Trajectory backtopickup3 = drive.trajectoryBuilder(comebackpose)
                .lineTo(new Vector2d(97, 62)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();


        waitForStart();

        if (isStopRequested()) return;

       //first back to drop the first specimen
        panningServo.moveSpecificPos(.35);
        drive.followTrajectory(firstspec);
        //ENTIRE BEAUTIFUL SPECIMEN SCORING CODE
        panningServo.moveSpecificPos(.35);
        panningMotor.rotateForward(1, 300);
        panningServo.moveSpecificPos(0);
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        panningMotor.rotateForward(-0.8, 500);
        panningServo.moveForwardMAX();

        // push in second and third specimens from ground into zone
        drive.followTrajectory(alligntopush1);
        drive.followTrajectory(alligntopush2);
        drive.followTrajectory(alligntopush3);
        drive.setPoseEstimate(pushPose);
        drive.followTrajectory(pushfirst);
        claw.moveSpecificPos(0.3);
        drive.followTrajectory(backfirst);
        panningServo.moveSpecificPos(0.4);
        drive.followTrajectory(flafe);
        drive.followTrajectory(pushsecond);
        //pick up second one off the wall
        claw.moveForwardMAX();
        sleep(350);
        panningServo.moveBackwardMIN();
        orientation.moveForwardMAX();
        //go to bar hang second one
        drive.setPoseEstimate(startcycling);
        panningServo.moveSpecificPos(.3);
        drive.followTrajectory(toBara);
        panningMotor.rotateForward(1, 400);
        panningServo.moveSpecificPos(0);
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        orientation.moveBackwardMIN();
        panningMotor.rotateForward(-0.8, 500);
        panningServo.moveForwardMAX();
        drive.setPoseEstimate(comebackpose);
        claw.moveSpecificPos(0.3);
        panningServo.moveSpecificPos(0.4);
        drive.followTrajectory(backtopickup);

        //paste (3rd block)
        claw.moveForwardMAX();
        sleep(350);
        panningServo.moveBackwardMIN();
        orientation.moveForwardMAX();
        //go to bar hang second one
        drive.setPoseEstimate(startcycling2);

        drive.followTrajectory(toBarb);
        panningServo.moveSpecificPos(.3);
        panningMotor.rotateForward(1, 400);
        panningServo.moveSpecificPos(0);
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        orientation.moveBackwardMIN();
        panningMotor.rotateForward(-0.8, 500);
        panningServo.moveForwardMAX();
        drive.setPoseEstimate(comebackpose);
        claw.moveSpecificPos(0.3);
        panningServo.moveSpecificPos(0.4);
        drive.followTrajectory(backtopickup3);

        //paste (4th block)
        claw.moveForwardMAX();
        sleep(350);
        panningServo.moveBackwardMIN();
        orientation.moveForwardMAX();
        //go to bar hang second one
        drive.setPoseEstimate(startcycling3);
        drive.followTrajectory(toBarc);
        panningServo.moveSpecificPos(.3);
        panningMotor.rotateForward(1, 400);
        panningServo.moveSpecificPos(0);
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        orientation.moveBackwardMIN();
        panningMotor.rotateForward(-0.8, 500);
        panningServo.moveForwardMAX();
        drive.setPoseEstimate(comebackpose);
        claw.moveSpecificPos(0.3);
        panningServo.moveSpecificPos(0.4);
    }
}
