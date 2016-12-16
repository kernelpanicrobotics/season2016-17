package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;

/**
 * Created by judenki on 11/19/16.
 */

public class KernelPanicPlatform {

    public DcMotor  leftMotorFront   = null;
    public DcMotor  rightMotorFront  = null;
    public DcMotor  leftMotorBack   = null;
    public DcMotor  rightMotorBack  = null;
    public GyroSensor gyro = null;

    // For unknown reasons must convert the 8-bit address programmed by the MR tool to a
    // 7-bit address.  This is effectively a divide by two.
    public static final int COLOR_SENSOR_BOTTOM_ADDRESS = 0x3c>>1;
    public static final int COLOR_SENSOR_SIDE_ADDRESS = 0x3e>>1;
    public ColorSensor colorBottom = null;
    public ColorSensor colorSide = null;
    public I2cDevice range = null;
    public I2cDeviceSynch rangeReader = null;
    //public Servo frontServo = null;
    //public Servo backServo = null;
    public CRServo frontServo = null;
    public CRServo backServo  = null;



    public final static double SERVO_EXTEND_POWER  = 254;
    public final static double SERVO_RETRACT_POWER = 254;
    public final static double SERVO_STOP_POWER = 0.00;


    public void init(HardwareMap ahwMap) {


        System.out.println("Showing the dcMotor hwMap");
        for (Map.Entry<String, DcMotor> i : ahwMap.dcMotor.entrySet()) {
            System.out.println("motor key: " + i.getKey());
            System.out.println("motor value: " + i.getValue().toString());
        }
        System.out.println("Done showing the dcMotor hwMap");
        rightMotorFront = ahwMap.dcMotor.get("right motor front");
        leftMotorFront = ahwMap.dcMotor.get("left motor front");
        rightMotorBack = ahwMap.dcMotor.get("right motor back");
        leftMotorBack = ahwMap.dcMotor.get("left motor back");

        //Gyro Sensor
        gyro = ahwMap.gyroSensor.get("gyro");
        gyro.calibrate();
        SystemClock.sleep(50);  // Give a breif moment so the init can start
        while (gyro.isCalibrating()  == true) {
            // need to add break out
        }

        //configure 2 color sensors
        colorSide = ahwMap.colorSensor.get("color side");
        colorSide.setI2cAddress(I2cAddr.create7bit(COLOR_SENSOR_SIDE_ADDRESS));
        colorSide.enableLed(false);
        colorBottom = ahwMap.colorSensor.get("color bottom");
        colorBottom.setI2cAddress(I2cAddr.create7bit(COLOR_SENSOR_BOTTOM_ADDRESS));
        colorBottom.enableLed(true);
/*
        //configure range sensor
        range = ahwMap.i2cDevice.get("range");
        rangeReader = new I2cDeviceSynchImpl(range, I2cAddr.create8bit(0x28), false);
        rangeReader.engage();
        // rangeValue[] = rangeReader.read(0x04, 2);  //Read 2 bytes starting at 0x04
        //Make method and wrapper class for all of this
*/

        //Configure Servos  tweak them in and out to confirm operation
        frontServo = ahwMap.crservo.get("frontServo");
        backServo = ahwMap.crservo.get("backServo");

        frontServo.setDirection(CRServo.Direction.FORWARD);
        backServo.setDirection(CRServo.Direction.FORWARD);
        frontServo.setPower(SERVO_EXTEND_POWER);
        backServo.setPower(SERVO_EXTEND_POWER);
        SystemClock.sleep(500);
        frontServo.setPower(SERVO_STOP_POWER);
        backServo.setPower(SERVO_STOP_POWER);
        frontServo.setDirection(CRServo.Direction.REVERSE);
        backServo.setDirection(CRServo.Direction.REVERSE);
        frontServo.setPower(SERVO_RETRACT_POWER);
        backServo.setPower(SERVO_RETRACT_POWER);
        SystemClock.sleep(500);
        frontServo.setPower(SERVO_STOP_POWER);
        backServo.setPower(SERVO_STOP_POWER);




    }


}
