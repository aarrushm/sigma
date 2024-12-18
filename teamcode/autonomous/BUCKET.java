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
public class BUCKET extends LinearOpMode {
    public static double DISTANCE = 24; // in

    public Slide slide;
    public Motors panningMotor;
    public Servos claw;
    public Servos panningServo;
    public Servos orientation;
    public Motors pulley;
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
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        Trajectory dropfirst = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-21,-11)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();
        Trajectory pickupA = drive.trajectoryBuilder(dropfirst.end())
                .lineTo(new Vector2d(-5,-10)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();
        Trajectory pickupB = drive.trajectoryBuilder(pickupA.end())
                .lineTo(new Vector2d(-14,-11)
                        ,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(100) )
                .build();

        waitForStart();

        if (isStopRequested()) return;

        //lineto and drop first specimen
        drive.followTrajectory(dropfirst);
        panningMotor.rotateForward(0.8, 300);
        panningServo.moveForwardMAX();
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        drive.turn(Math.toRadians(-30));
        sleep(500);
        panningServo.moveSpecificPos(.35);
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.zero);
        panningServo.moveForwardMAX();
        sleep(1000);
        panningMotor.rotateForward(-0.8, 500);
        drive.turn(Math.toRadians(30));
        slide.MoveToLevel(Slide.level.pan_highbar);
        sleep(500);
        claw.moveForwardMAX();
        sleep(700);
        panningServo.moveSpecificPos(.35);
        slide.MoveToLevelAsync(Slide.level.zero);
        sleep(1000);
        panningMotor.rotateForward(1, 300);
        panningServo.moveSpecificPos(.35);
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        sleep(500);
        drive.turn(Math.toRadians(-40));
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.zero);
        panningServo.moveForwardMAX();
        sleep(1000);
        panningMotor.rotateForward(-0.8, 500);
        drive.turn(Math.toRadians(40));
        drive.followTrajectory(pickupA);
        slide.MoveToLevel(Slide.level.pan_highbar);
        sleep(500);
        claw.moveForwardMAX();
        sleep(700);
        panningServo.moveSpecificPos(.35);
        slide.MoveToLevelAsync(Slide.level.zero);
        drive.followTrajectory(pickupB);
        panningServo.moveForwardMAX();
        drive.turn(Math.toRadians(-40));
        panningMotor.rotateForward(1, 300);
        slide.MoveToLevel(Slide.level.slide_highbucket);
        panningServo.moveSpecificPos(.35);
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.zero);
        panningServo.moveForwardMAX();
        sleep(1000);
        panningMotor.rotateForward(-0.8, 500);
        drive.turn(Math.toRadians(65));
        claw.moveSpecificPos(0.5);
        slide.MoveToLevel(Slide.level.panigging);
        sleep(500);
        claw.moveForwardMAX();
        sleep(700);
        slide.MoveToLevelAsync(Slide.level.zero);
        panningServo.moveSpecificPos(.35);
        sleep(1000);
        panningMotor.rotateForward(1, 300);
        panningServo.moveSpecificPos(.35);
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        drive.turn(Math.toRadians(-60));
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.zero);
        panningServo.moveForwardMAX();
        sleep(1000);
    }
}
