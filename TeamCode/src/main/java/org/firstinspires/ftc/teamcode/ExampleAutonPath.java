package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import helperClasses.StateMachine;

@Autonomous
public class ExampleAutonPath extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        StateMachine robotState = new StateMachine(leftFront, leftBack, rightFront, rightBack, this, StateMachine.DriveTrain.DRIVE_TRAIN_MECANUM);
        waitForStart();

        //this completes a complete square shaped path
        robotState.runState(StateMachine.State.FORWARDS, 30); //each side of this square path measures 30cm
        robotState.runState(StateMachine.State.LEFTSTRAFE, 30);
        robotState.runState(StateMachine.State.BACKWARDS, 30);
        robotState.runState(StateMachine.State.RIGHTSTRAFE, 30);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}