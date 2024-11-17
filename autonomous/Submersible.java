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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive2;

@Config
@Autonomous(group = "drive")
public class Submersible extends LinearOpMode {
    public static double DISTANCE = 24; // in

    public Slides slide;
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
        slide = new Slides(hardwareMap, "slide", 6000);
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
        panningServo.moveBackwardMIN();
        orientation.moveForwardMAX();


        SampleMecanumDrive2 drive = new SampleMecanumDrive2(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory tohang = drive.trajectoryBuilder(startPose)
                .back(22.7)
                .build();
        Trajectory firstback = drive.trajectoryBuilder(startPose)
                .back(25)
                .build();
        Trajectory forward = drive.trajectoryBuilder(startPose)
                .forward(45)
                .build();
        Trajectory backward1 = drive.trajectoryBuilder(startPose)
                .back(20)
                .build();
        Trajectory backward2 = drive.trajectoryBuilder(startPose)
                .back(14)
                .build();
        Trajectory grabclip = drive.trajectoryBuilder(startPose)
                .forward(20)
                .build();
        Trajectory turn = drive.trajectoryBuilder(startPose)
                .strafeLeft(24.3)
                .build();
        Trajectory park = drive.trajectoryBuilder(startPose)
                .back(20)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(tohang);
        panningMotor.rotateForward(0.8, 300);
        slide.runForward(1.0, 1000);
        claw.moveBackwardMIN();
        sleep(500);
        slide.runBackward(1.0, 900);
        panningMotor.rotateForward(-0.8, 300);
        drive.turn(4);
        drive.followTrajectory(firstback);
        drive.turn(2.3);
        drive.followTrajectory(forward);
        drive.followTrajectory(backward1);
        panningServo.moveForwardMAX();
        sleep(2000);
        drive.followTrajectory(grabclip);
        claw.moveForwardMAX();
        sleep(500);
        panningServo.moveBackwardMIN();
        drive.followTrajectory(backward2);
        drive.turn(-6.4);
        panningMotor.rotateForward(0.8, 300);
        slide.runForward(1.0, 1000);
        claw.moveBackwardMIN();
        sleep(500);
        slide.runBackward(1.0, 900);
        panningServo.moveForwardMAX();
        drive.turn(5.2);
        drive.followTrajectory(turn);
        drive.turn(4);
        drive.followTrajectory(park);
        panningServo.moveBackwardMIN();
        sleep(1000);
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

    public void strafeRight(int timeMs) {
        fl.setPower(1.0); // forward
        fr.setPower(-1.0); // backward
        bl.setPower(1.0); // backward
        br.setPower(-1.0); // forward
        if (timeMs > 0) {
            sleep(timeMs);
            stopMovement();
        }
    }

    public void moveForward(int timeMs) {
        fl.setPower(1.0); // forward
        fr.setPower(1.0); // backward
        bl.setPower(1.0); // backward
        br.setPower(1.0); // forward
        if (timeMs > 0) {
            sleep(timeMs);
            stopMovement();
        }
    }

    public void moveBackward(int timeMs) {
        fl.setPower(-1.0); // forward
        fr.setPower(-1.0); // backward
        bl.setPower(-1.0); // backward
        br.setPower(-1.0); // forward
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
