package helperClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveStateMachine {

    private final DcMotor m_leftFront; //These cannot be changed into other motors once assigned in the constructor
    private final DcMotor m_leftBack;
    private final DcMotor m_rightFront;
    private final DcMotor m_rightBack;



    private LinearOpMode m_opMode;

    static final double     COUNTS_PER_MOTOR_REV = 1478.4 ;    // Number of ticks for every full revolution/rotation of the motor shaft
    static final double     DRIVE_GEAR_REDUCTION = 1.0 ;     // Depends on gearing ratio between motor and wheel
    static final double     WHEEL_DIAMETER_CM = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);  //This is the amount of ticks we have every cm travelled by the wheel
    static final double     DRIVE_SPEED = 0.6;
    static final double     TURN_SPEED = 0.5;



    public DriveStateMachine(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, LinearOpMode opMode) { //the constructor

        //the motors are assigned
        m_leftFront = leftFront;
        m_leftBack = leftBack;
        m_rightFront = rightFront;
        m_rightBack = rightBack;

        m_opMode = opMode;

    }



    public enum State {
        INITIALISE,
        FORWARDS,
        BACKWARDS,
        LEFTSTRAFE,
        RIGHTSTRAFE,
        TURNLEFT,
        TURNRIGHT,
        ARMUP,
        ARMDOWN,
        FLIPSERVO,
        BOTDOWN,
        CHECK,
        STOP
    };

    public void runState(State currState, double distanceCm)
    {
        //this function is called
        switch(currState)
        {
            case FORWARDS:
                encoderDrive(DRIVE_SPEED, distanceCm, distanceCm, distanceCm, distanceCm);
                break;
            case BACKWARDS:
                encoderDrive(DRIVE_SPEED, -distanceCm, -distanceCm, -distanceCm, -distanceCm);
                break;
            case LEFTSTRAFE:
                encoderDrive(DRIVE_SPEED, distanceCm, -distanceCm, -distanceCm, distanceCm);
                break;
            case RIGHTSTRAFE:
                encoderDrive(DRIVE_SPEED, -distanceCm, distanceCm, distanceCm, -distanceCm);
                break;
            case TURNLEFT:
                encoderDrive(TURN_SPEED, -distanceCm, -distanceCm, distanceCm, distanceCm);
                break;
            case TURNRIGHT:
                encoderDrive(TURN_SPEED, distanceCm, distanceCm, -distanceCm, -distanceCm);
                break;
        }
    }


    private void encoderDrive(double speed,
                             double leftFrontCm, double leftBackCm, double rightFrontCm, double rightBackCm) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;


        // Ensure that the opmode is still active
        if (m_opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = m_leftFront.getCurrentPosition() + (int)(leftFrontCm * COUNTS_PER_CM);
            newLeftBackTarget = m_leftBack.getCurrentPosition() + (int)(leftBackCm * COUNTS_PER_CM);
            newRightFrontTarget = m_rightFront.getCurrentPosition() + (int)(rightFrontCm * COUNTS_PER_CM);
            newRightBackTarget = m_rightBack.getCurrentPosition() + (int)(rightBackCm * COUNTS_PER_CM);

            m_leftFront.setTargetPosition(newLeftFrontTarget);
            m_leftBack.setTargetPosition(newLeftBackTarget);
            m_rightFront.setTargetPosition(newRightFrontTarget);
            m_rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            m_leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m_rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            m_leftFront.setPower(Math.abs(speed));
            m_leftBack.setPower(Math.abs(speed));
            m_rightFront.setPower(Math.abs(speed));
            m_rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and both motors are still running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            while (m_opMode.opModeIsActive() &&
                    (m_leftFront.isBusy() && m_leftBack.isBusy() && m_rightFront.isBusy() && m_rightBack.isBusy())) {

                // Display it for the driver.
                m_opMode.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newLeftBackTarget, newRightFrontTarget,  newRightBackTarget);
                m_opMode.telemetry.addData("Path2",  "Running at %7d :%7d",
                        m_leftFront.getCurrentPosition(),
                        m_leftBack.getCurrentPosition(),
                        m_rightFront.getCurrentPosition(),
                        m_leftBack.getCurrentPosition());
                m_opMode.telemetry.update();
            }

            // Stop all motion;

            m_leftFront.setPower(0);
            m_leftBack.setPower(0);
            m_rightFront.setPower(0);
            m_rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            m_leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m_leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m_rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m_rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            m_opMode.sleep(250);   // optional pause after each move
        }
    }

}


