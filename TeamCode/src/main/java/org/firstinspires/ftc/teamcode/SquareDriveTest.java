package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by judenki on 11/19/16.
 */
@Autonomous(name="Square Drive Test", group="Kernel-Panic")
public class SquareDriveTest extends LinearOpMode {
    KernelPanicPlatform robot = new KernelPanicPlatform();

    @Override
    public void runOpMode() throws InterruptedException {
        int[] Headings = {
                45, 315, 90, 80
        };
        int[] rotationDir = {
                1, -1, 1, 1
        };
        int leg = 0;
        robot.init(hardwareMap);
        telemetry.addData("heading", robot.gyro.getHeading());
        telemetry.update();

        waitForStart();
        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotorFront, robot.leftMotorBack };
        DcMotor[] rightMotors = new DcMotor[]{robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);


        myDrive.setParams(12.5, 2, 79.5, .2, .15, .12, 1, -1, robot.gyro, this);

        // Move Forward
        myDrive.moveForward(12, 0.6);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        for (leg = 0; leg <= 3; leg++) {
        // Right turn with new > current  go from 0 to 90
            myDrive.gyroTurn2(Headings[leg], rotationDir[leg]);
            myDrive.allStop();
            //Heading should now be 90
            telemetry.addData("heading", robot.gyro.getHeading());
            telemetry.update();
            SystemClock.sleep(1000);  // Take a short nap so telemetry can be checked//
        }
/*

        // Left turn with new < current  go from 90 to 45
        myDrive.gyroTurn(45, myDrive.LEFT_TURN);
        myDrive.allStop();
        //Heading should now be 45
        telemetry.addData("heading", robot.gyro.getHeading());
        telemetry.update();
        SystemClock.sleep(1000);


        //Right turn with new < current   go from 45 to 0
        myDrive.gyroTurn(0, myDrive.RIGHT_TURN);
        myDrive.allStop();
        //Heading should now be 0
        telemetry.addData("heading", robot.gyro.getHeading());
        telemetry.update();
        SystemClock.sleep(1000);

        //Reports correct heading but robot is off.   Try no calibrating gyro at start????  Not gyro
        //calibration issue at start.  Move gyro from back upper right to center of platform.  Appears to just
        //be a gyro thing.   Compensate by only taking short turns.



        // Left turn with new > current  Turn from 0 to 270
        myDrive.gyroTurn(270, myDrive.LEFT_TURN);
        myDrive.allStop();
        //Heading should now be 270
        telemetry.addData("heading", robot.gyro.getHeading());
        telemetry.update();
        SystemClock.sleep(1000);

*/
        // Large gyro turns while passing through sero seem to cause some not nice behaviour.
        //Avoid large turns through zero if possible.


        while(opModeIsActive()) {
            idle();
        }
    }
}
