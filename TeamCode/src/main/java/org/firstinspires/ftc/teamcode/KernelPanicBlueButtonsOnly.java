package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;



/**
 * Created by Kernel Panic on 11/26/16.
 *
 * DO NOT TOUCH
 * DO NOT TOUCH
 * DO NOT TOUCH
 * DO NOT TOUCH
 * DO NOT TOUCH
 * DO NOT TOUCH
 * DO NOT TOUCHDO NOT TOUCH
 * DO NOT TOUCH
 * DO NOT TOUCH
 * DO NOT TOUCH
 * DO NOT TOUCH
 *
 * Code is as good as it is going to get for first competition.   Leave it alone!!!!!
 * This means you!!!
 *
 */
@Autonomous(name="Kernel Panic Blue Buttons Only", group="Kernel Panic")
public class KernelPanicBlueButtonsOnly extends LinearOpMode {
    KernelPanicPlatform robot = new KernelPanicPlatform();

    @Override
    public void runOpMode() throws InterruptedException {
        int currentHeading;
        int nextHeading;
        //public static final int COLOR_SENSOR_BOTTOM_ADDRESS = 0x3c>>1;
        //public ColorSensor colorBottom = null;
        boolean continueForward = true;


        robot.init(hardwareMap);


        //Clean this up because of drive class
        DcMotor[] leftMotors = new DcMotor[]{robot.leftMotorFront, robot.leftMotorBack};
        DcMotor[] rightMotors = new DcMotor[]{robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);
        myDrive.setParams(12.5, 2, 79.5, .18, .15, .12, 1, -1, robot.gyro, this);
        waitForStart();

        opModeIsActive();

        // Move forward some
        myDrive.moveForward(12, 0.3);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();

        }
        myDrive.allStop();

        //Try to turn 45 degrees

        myDrive.gyroTurn(53, myDrive.RIGHT_TURN);
        myDrive.allStop();


        myDrive.moveForward(44, 0.3);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        myDrive.allStop();


        myDrive.gyroTurn(2, myDrive.LEFT_TURN);  // Seems to lose its mind and think 0 is off by 2 to 5
        // May be a function of battery power
        myDrive.allStop();


        long mytime = System.currentTimeMillis();
        long loopingtime = 0;
        myDrive.driveMove(.1, 0);
        boolean keepmoving = true;
        while ((keepmoving == true) && opModeIsActive()) {
            if (((robot.colorBottom.red() > 20) &&
                    (robot.colorBottom.blue() > 20) &&
                    (robot.colorBottom.green() > 20)) ||
                    (loopingtime > 5000)) {
                myDrive.allStop();
                keepmoving = false;
            }
            loopingtime = System.currentTimeMillis() - mytime;
        }
        myDrive.allStop();

        myDrive.moveForward(2, 0.1);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        myDrive.allStop();


        //Red Autonomous  -- servo positions are probably swapped
        //dataDump();
        //SystemClock.sleep(1000); //Give time to look at data
        if (robot.colorSide.red() > robot.colorSide.blue()) {
            robot.backServo.setDirection(CRServo.Direction.FORWARD);
            robot.backServo.setPower(robot.SERVO_EXTEND_POWER);
            safeSleep(2000);
            robot.backServo.setDirection(CRServo.Direction.REVERSE);
            robot.backServo.setPower(robot.SERVO_RETRACT_POWER);
            safeSleep(2000);
            robot.backServo.setPower(robot.SERVO_STOP_POWER);
        } else if (robot.colorSide.blue() > robot.colorSide.red()){
            robot.frontServo.setDirection(CRServo.Direction.FORWARD);
            robot.frontServo.setPower(robot.SERVO_EXTEND_POWER);
            safeSleep(2000);
            robot.frontServo.setDirection(CRServo.Direction.REVERSE);
            robot.frontServo.setPower(robot.SERVO_RETRACT_POWER);
            safeSleep(2000);
            robot.frontServo.setPower(robot.SERVO_STOP_POWER);
        }
        else{
            //Do nothing
        }

        robot.frontServo.setPower(robot.SERVO_STOP_POWER);
        robot.backServo.setPower(robot.SERVO_STOP_POWER);
        //    mytime = System.currentTimeMillis();
        //    while ((System.currentTimeMillis() - mytime < 1000) && opModeIsActive()) ;


        mytime = System.currentTimeMillis();
        loopingtime = 0;
        myDrive.driveMove(.1, 0);
        keepmoving = true;
        while ((keepmoving == true) && opModeIsActive()) {
            if (((robot.colorBottom.red() > 20) &&
                    (robot.colorBottom.blue() > 20) &&
                    (robot.colorBottom.green() > 20)) ||
                    (loopingtime > 5000)) {
                myDrive.allStop();
                keepmoving = false;
            }
            loopingtime = System.currentTimeMillis() - mytime;
        }
        myDrive.allStop();

        myDrive.moveForward(2, 0.1);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        myDrive.allStop();

        //Blue Autonomous
        //dataDump();
        //SystemClock.sleep(1000); //Give time to look at data
        if (robot.colorSide.red() > robot.colorSide.blue()) {
            robot.backServo.setDirection(CRServo.Direction.FORWARD);
            robot.backServo.setPower(robot.SERVO_EXTEND_POWER);
            safeSleep(2000);
            robot.backServo.setDirection(CRServo.Direction.REVERSE);
            robot.backServo.setPower(robot.SERVO_RETRACT_POWER);
            safeSleep(2000);
            robot.backServo.setPower(robot.SERVO_STOP_POWER);
        } else if (robot.colorSide.blue() > robot.colorSide.red()){
            robot.frontServo.setDirection(CRServo.Direction.FORWARD);
            robot.frontServo.setPower(robot.SERVO_EXTEND_POWER);
            safeSleep(2000);
            robot.frontServo.setDirection(CRServo.Direction.REVERSE);
            robot.frontServo.setPower(robot.SERVO_RETRACT_POWER);
            safeSleep(2000);
            robot.frontServo.setPower(robot.SERVO_STOP_POWER);
        }
        else{
            //Do nothing
        }

        robot.frontServo.setPower(robot.SERVO_STOP_POWER);
        robot.backServo.setPower(robot.SERVO_STOP_POWER);

        //mytime = System.currentTimeMillis();
        //while ((System.currentTimeMillis() - mytime < 1000)  && opModeIsActive()) ;


        //Turn towards center and drive to knock off ball
        //dataDump();
        //SystemClock.sleep(3000); //Give time to look at data
/*
        myDrive.gyroTurn(220, myDrive.RIGHT_TURN);  //Left turn near zero has a problem
        myDrive.allStop();
        myDrive.moveForward(66, 0.3);
        while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
            myDrive.update();
        }
        */
        myDrive.allStop();










        while(opModeIsActive()) {
            idle();
        }   // New opModeActive()

        myDrive.allStop();
        robot.frontServo.setPower(robot.SERVO_STOP_POWER);
        robot.backServo.setPower(robot.SERVO_STOP_POWER);

    }

    public int newHeading (int currentHeading, int turnHeading) {
        int tempHeading;

        tempHeading = currentHeading + turnHeading;

        if (tempHeading > 360)
            tempHeading = tempHeading -360;

        if (tempHeading < 0)
            tempHeading = tempHeading + 360;

        return (tempHeading);


    }

    public void dataDump() {
        telemetry.addData("heading", robot.gyro.getHeading());
        telemetry.addData("Side      Red   ", "%d", robot.colorSide.red());
        telemetry.addData("Side      Green ", "%d", robot.colorSide.green());
        telemetry.addData("Side      Blue  ", "%d", robot.colorSide.blue());
        telemetry.addData("heading", robot.gyro.getHeading());
        telemetry.update();
    }

    public void safeSleep (long duration) {
        long currentTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - duration < currentTime) && opModeIsActive()) {
            //kill some time
            // idle();
        }
    }


}
