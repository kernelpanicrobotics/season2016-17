package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;


/**
 * Created by Kernel Panic on 11/26/16.
 */
@Autonomous(name="Kernel Panic Autonomous Blue", group="Kernel Panic")
public class KernelPanicAutonomousBlue extends LinearOpMode {
    //Juden Ki Launching Robot
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
        myDrive.setParams(12.5, 2, 79.5, .2, .15, .12, 1, -1, robot.gyro, this);
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


            myDrive.moveForward(41, 0.3);
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
            dataDump();
            SystemClock.sleep(3000); //Give time to look at data
            if ((robot.colorSide.red() > 1) && (robot.colorSide.red() > robot.colorSide.blue())) {
                robot.frontServo.setPosition(robot.SERVO_MAX_RANGE_FRONT);
            } else {
                robot.backServo.setPosition(robot.SERVO_MAX_RANGE_BACK);
            }
            mytime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - mytime < 1000) && opModeIsActive()) ;
            robot.frontServo.setPosition(robot.SERVO_MIN_RANGE_FRONT);
            robot.backServo.setPosition(robot.SERVO_MIN_RANGE_BACK);

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

            //Red Autonomous
            dataDump();
            SystemClock.sleep(3000); //Give time to look at data
        if ((robot.colorSide.red() > 1) && (robot.colorSide.red() > robot.colorSide.blue())) {
                robot.frontServo.setPosition(robot.SERVO_MAX_RANGE_FRONT);
            } else {
                robot.backServo.setPosition(robot.SERVO_MAX_RANGE_BACK);
            }
            ;
            mytime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - mytime < 1000)  && opModeIsActive()) ;
            robot.frontServo.setPosition(robot.SERVO_MIN_RANGE_FRONT);
            robot.backServo.setPosition(robot.SERVO_MIN_RANGE_BACK);

            //Turn towards center and drive to knock off ball
            myDrive.gyroTurn(225, myDrive.LEFT_TURN);
            myDrive.allStop();
            myDrive.moveForward(66, 0.3);
            while ((myDrive.motorsRunning() == true) && opModeIsActive()) {
                myDrive.update();
            }
            myDrive.allStop();




        while(opModeIsActive()) {
            idle();
        }   // New opModeActive()

        myDrive.allStop();

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
        telemetry.update();
    }


}
