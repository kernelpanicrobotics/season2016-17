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
@Autonomous(name="Kernel Panic Autonomous Red", group="Kernel Panic")
public class KernelPanicAutonomousRed extends LinearOpMode {
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
        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotorFront, robot.leftMotorBack };
        DcMotor[] rightMotors = new DcMotor[]{ robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);
        myDrive.setParams(12.5, 2, 79.5, .3, .2, .12, 1, -1, robot.gyro, this);
        waitForStart();

        // Move forward some
        myDrive.moveForward(-12, 0.3);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        //Try to turn 45 degrees
        currentHeading = robot.gyro.getHeading();
        nextHeading = newHeading(currentHeading, -42);
        myDrive.driveMove(0, -0.25);
        while(robot.gyro.getHeading() <= nextHeading) {
            //Kill some time



            //telemetry.addData("heading", robot.gyro.getHeading());
            //telemetry.update();
        }
        myDrive.allStop();

        myDrive.moveForward(-47, 0.3);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        currentHeading = robot.gyro.getHeading();
        nextHeading = newHeading(currentHeading, 42);
        myDrive.driveMove(0, 0.25);
        while(robot.gyro.getHeading() >= nextHeading) {
            //Kill some time
            //telemetry.addData("heading", robot.gyro.getHeading());
            //telemetry.update();
        }
        myDrive.allStop();

       /* myDrive.moveForward(30, 0.1);
        while ((myDrive.motorsRunning() == true)&&
                (continueForward == true)){
            myDrive.update();


        }*/
        long mytime = System.currentTimeMillis();
        long loopingtime = 0;
        myDrive.driveMove(-0.1,0);
        boolean keepmoving = true;
        while (keepmoving == true) {
            if (((robot.colorBottom.red() > 20) &&
                    (robot.colorBottom.blue() > 20) &&
                    (robot.colorBottom.green() > 20)) ||
                    (loopingtime > 5000)){
                myDrive.allStop();
                keepmoving = false;
            }
            loopingtime = System.currentTimeMillis() - mytime;

        }

        myDrive.moveForward(-4, 0.1);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }


        //Red Autonomous
        if(robot.colorSide.red() > 1) {
            robot.frontServo.setPosition(10);
        }
        else{
            robot.backServo.setPosition(10);
        }
        telemetry.addData("Side      Red   ", "%d", robot.colorSide.red());
        telemetry.addData("Side      Green ", "%d", robot.colorSide.green());
        telemetry.addData("Side      Blue  ", "%d", robot.colorSide.blue());
        mytime = System.currentTimeMillis();
        while(System.currentTimeMillis() - mytime < 1000);
        robot.frontServo.setPosition(0);
        robot.backServo.setPosition(0);

        mytime = System.currentTimeMillis();
        loopingtime = 0;
        myDrive.driveMove(-0.1,0);
        keepmoving = true;
        while (keepmoving == true) {
            if (((robot.colorBottom.red() > 20) &&
                    (robot.colorBottom.blue() > 20) &&
                    (robot.colorBottom.green() > 20)) ||
                    (loopingtime > 5000)){
                myDrive.allStop();
                keepmoving = false;
            }
            loopingtime = System.currentTimeMillis() - mytime;

        }

        myDrive.moveForward(-4, 0.1);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        //Red Autonomous
        if(robot.colorSide.red() > 1) {
            robot.frontServo.setPosition(10);
        }
        else{
            robot.backServo.setPosition(10);
        }
        telemetry.addData("Side      Red   ", "%d", robot.colorSide.red());
        telemetry.addData("Side      Green ", "%d", robot.colorSide.green());
        telemetry.addData("Side      Blue  ", "%d", robot.colorSide.blue());
        mytime = System.currentTimeMillis();
        while(System.currentTimeMillis() - mytime < 1000);
        robot.frontServo.setPosition(0);
        robot.backServo.setPosition(0);

        currentHeading = robot.gyro.getHeading();
        nextHeading = newHeading(currentHeading, 135);
        myDrive.driveMove(0, 0.25);
        while((robot.gyro.getHeading()  != nextHeading) ||
                (robot.gyro.getHeading() != nextHeading+1) ||
                (robot.gyro.getHeading() != nextHeading+2) ||
                (robot.gyro.getHeading() != nextHeading-1) ||
                (robot.gyro.getHeading() != nextHeading-2))
        {
            //Kill some time
            //telemetry.addData("heading", robot.gyro.getHeading());
            //telemetry.addData("next heading", nextHeading);
            //telemetry.update();
        }
        while(robot.gyro.getHeading() >= nextHeading){

        }
        myDrive.allStop();


        myDrive.moveForward(-66, 0.3);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        while(opModeIsActive()) {
            idle();
        }

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


}
