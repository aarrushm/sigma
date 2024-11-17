package org.firstinspires.ftc.teamcode.autonomous;
//6.92307692308 is 1 inch
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive1;

@Config
@Autonomous(group = "drive")
public class Bucket extends LinearOpMode {
    public static double DISTANCE = 24; // in

    public Slide slide;
    public Motors panningMotor;
    public Servos claw;
    public Servos panningServo;
    public Servos orientation;
    public Motors pulley;
    public HuskyLenses frontLens;
    public DcMotor fl;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor br;

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
        frontLens = new HuskyLenses(hardwareMap, "frontlens", "color");

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        runFrames = 0;
        claw.moveForwardMAX();
        panningServo.moveForwardMAX();
        orientation.moveForwardMAX();


        SampleMecanumDrive1 drive = new SampleMecanumDrive1(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory firstturn = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(10, 25.4, Math.toRadians(0)))
                .build();
        Trajectory backward = drive.trajectoryBuilder(startPose)
                .back(10.5)
                .build();
        Trajectory backward1 = drive.trajectoryBuilder(startPose)
                .back(4.5)
                .build();
        Trajectory backward2 = drive.trajectoryBuilder(startPose)
                .back(12.5)
                .build();
        Trajectory lineTo2 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(16, 0, Math.toRadians(0)))
                .build();
        Trajectory lineTo3 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-4, -14, Math.toRadians(0)))
                .build();
        Trajectory lineTo6 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-16, 0, Math.toRadians(0)))
                .build();
        Trajectory lineTo7 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(0, -13, Math.toRadians(0)))
                .build();
        Trajectory lineTo4 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(0, 16, Math.toRadians(0)))
                .build();
        Trajectory lineTo5 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(19, 0, Math.toRadians(0)))
                .build();
        Trajectory backward17 = drive.trajectoryBuilder(startPose)
                .back(12)
                .build();
        Trajectory forward = drive.trajectoryBuilder(startPose)
                .forward(36)
                .build();
        Trajectory thosewhogrimace = drive.trajectoryBuilder(startPose)
                .forward(24.3)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        //following is the BASE DEPOSIT CODE
        orientation.moveForwardMAX();
        panningMotor.rotateForward(0.8, 300);
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        //remove from base deposit code
        drive.followTrajectory(backward);
        sleep(500);
        panningServo.moveBackwardMIN();
        sleep(500);
        claw.moveBackwardMIN();
        sleep(500);
        panningServo.moveForwardMAX();
        sleep(500);
        // I MADE THIS ASYNC
        slide.MoveToLevelAsync(Slide.level.zero);
        drive.followTrajectory(firstturn);
        panningMotor.rotateForward(-1.0, 300);
        sleep(500);

        // First turn
        frontLens.updateObservedObjects();
        firstBlock = frontLens.getFirstObject();
        if (firstBlock != null) {
            xDifference = Math.abs(xCenter - firstBlock.x);
            xMovement = scale1 * ((xDifference * firstBlock.width) / (firstBlock.width + xDifference));
            RobotLog.d("x value" + xMovement);
            drive.turn(xMovement);
        }

        drive.followTrajectory(thosewhogrimace);
        drive.turn(0.7);
        sleep(400);
        claw.moveForwardMAX();
        sleep(700);
        drive.turn(-0.7);
        panningMotor.rotateForward(0.8, 300);
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        drive.followTrajectory(backward);
        drive.followTrajectory(lineTo3);
        drive.followTrajectory(backward2);
        //following is the BASE DEPOSIT CODE
        orientation.moveForwardMAX();
        panningServo.moveBackwardMIN();
        sleep(500);
        claw.moveBackwardMIN();
        sleep(500);
        panningServo.moveForwardMAX();
        sleep(500);
        slide.MoveToLevelAsync(Slide.level.zero);
        drive.followTrajectory(lineTo4);
        panningMotor.rotateForward(-1.0, 300);
        frontLens.updateObservedObjects();
        firstBlock = frontLens.getFirstObject();
        if (firstBlock != null) {
            xDifference = Math.abs(xCenter - firstBlock.x);
            xMovement = scale2 * ((xDifference * firstBlock.width) / (firstBlock.width + xDifference));
            drive.turn(xMovement);
        }
        drive.followTrajectory(lineTo5);
        drive.turn(0.7);
        claw.moveForwardMAX();
        sleep(500);
        drive.turn(-0.7);
        panningMotor.rotateForward(0.8, 300);
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        drive.turn(-1.35);
        drive.followTrajectory(lineTo6);
        drive.followTrajectory(lineTo7);
        drive.followTrajectory(backward17);
        panningServo.moveBackwardMIN();
        sleep(500);
        claw.moveBackwardMIN();
        sleep(500);
        panningServo.moveForwardMAX();
        slide.MoveToLevelAsync(Slide.level.zero);
        sleep(1000);

//        drive.followTrajectory(forward);

    }

    public void strafeLeft(int timeMs) {
        fl.setPower(-1.0); // backward
        fr.setPower(1.0); // forward
        bl.setPower(-1.0); // forward
        br.setPower(1.0); // backward
        if (timeMs > 0) {
            sleep(timeMs);
            stopMovement();
        }
    }

    public void moveForwardSlightLeft(int timeMs) {
        fl.setPower(0); // forward
        fr.setPower(1.0); // forward
        bl.setPower(0); // forward
        br.setPower(1.0); // forward
        if (timeMs > 0) {
            sleep(timeMs);
            stopMovement();
        }
    }

        public void stopMovement() {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
    }
