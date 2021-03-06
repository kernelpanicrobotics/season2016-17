package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareK9bot
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public Servo    arm         = null;
    public CRServo  claw        = null;
    public CRServo  continuous  = null;
    public TouchSensor touch    = null;
    public IrSeekerSensor seeker = null;
    public ColorSensor color     = null;
    public OpticalDistanceSensor ods = null;
    public GyroSensor gyro = null;
    public boolean gyroInit = false;


    public final static double ARM_HOME = 0.0;
    public final static double CLAW_HOME = 0.0;
    public final static double ARM_MIN_RANGE  = 0.20;
    public final static double ARM_MAX_RANGE  = 0.90;
    public final static double CLAW_MIN_RANGE  = 0.20;
    public final static double CLAW_MAX_RANGE  = 0.7;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    HardwareDevice hd = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareK9bot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        //hwMap.dcMotor.put("right motor", new DcMotorImpl(, 1));
        //hwMap.dcMotor.put("left motor", leftMotor);
        // Define and Initialize Motors
        System.out.println("Showing the dcMotor hwMap");
        for(Map.Entry<String, DcMotor> i : hwMap.dcMotor.entrySet() ) {
            System.out.println("motor key: " + i.getKey());
            System.out.println("motor value: " + i.getValue().toString() );
        }
        System.out.println("Done showing the dcMotor hwMap");
        rightMotor  = hwMap.dcMotor.get("right motor foo");
        leftMotor   = hwMap.dcMotor.get("left motor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        //leftMotor.setPower(0);
        //rightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Define and initialize ALL installed servos.
        arm = hwMap.servo.get("arm");
        claw = hwMap.crservo.get("claw");

        //Initialize the servos May be redundant.
        arm.scaleRange(ARM_MIN_RANGE, ARM_MAX_RANGE);
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(ARM_HOME);

        claw.setDirection(CRServo.Direction.FORWARD);
        claw.setPower(0);//Full Stop



        //continuous = hwMap.servo.get("continuous");
        continuous = hwMap.crservo.get("continuous");
        continuous.setDirection(CRServo.Direction.FORWARD);
        continuous.setPower(0);//Full Stop


        // Touch Sensor
        touch = hwMap.touchSensor.get("touch");

        //IR Seeker sensor, require either a 600HZ or 1200HZ signal from playfield
        seeker = hwMap.irSeekerSensor.get("seeker");
        seeker.setMode(IrSeekerSensor.Mode.MODE_600HZ);

        //Color Sensor
        color = hwMap.colorSensor.get("color");
        color.enableLed(true);


        //Optical distance sensor
        ods = hwMap.opticalDistanceSensor.get("range");
        ods.enableLed(true);



        //Playing around with gyro sensor
        gyro = hwMap.gyroSensor.get("gyro");
        if(gyroInit == false) {
            gyro.calibrate();
            while (gyro.isCalibrating()  == true) {
                // need to add break out
            }
            gyroInit = true;
        }




        // To see motors run:
        //     - uncomment these lines
        //     - update controller program
        //     - connect to test bench
        //     -use configuration "test sam b"

        //leftMotor.setPower(0.5);
        //rightMotor.setPower(0.5);



    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
