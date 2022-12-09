package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Class Autonomous", group="Linear Opmode")
public class ClassAutonomous extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx slide = null;
    private Servo clawServo = null;


    public void wait(double milliseconds, double wait){
        while(runtime.milliseconds() < milliseconds + wait && opModeIsActive()){
        }
    }
//    public void turnRight(double power){
//
//        while(leftFrontDrive.getCurrentPosition() < 90 && opModeIsActive()){
//            leftFrontDrive.setPower(power);
//            rightFrontDrive.setPower(-power);
//            leftBackDrive.setPower(power);
//            rightBackDrive.setPower(-power);
//        }
//        stopMotors();
//    }
//    public void turnLeft(double power){
//        gyroscope.resetZAxisIntegrator();
//        while(gyroscope.rawZ() > -90 && opModeIsActive()){
//            leftFrontDrive.setPower(-power);
//            rightFrontDrive.setPower(power);
//            leftBackDrive.setPower(-power);
//            rightBackDrive.setPower(power);
//        }
//        stopMotors();
//    }
    public void goForward(int deltaPosition, int initialPosition, double power){
        while(leftFrontDrive.getCurrentPosition() < initialPosition + deltaPosition && opModeIsActive()){
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);
        }
        stopMotors();
    }
    public void goBackwards(int deltaPosition, int initialPosition, double power){
        while(leftFrontDrive.getCurrentPosition() > initialPosition - deltaPosition && opModeIsActive()){
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);
        }
        stopMotors();
    }
    public void stopMotors(){
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }
    public void extendSlide(double milliseconds){
        while(runtime.milliseconds() < milliseconds + 5000 && opModeIsActive()){
            if(slide.getCurrentPosition() > 0 || slide.getCurrentPosition() >= -2057){
                slide.setVelocity(-15 * 28);
            }
            else{
                slide.setVelocity(0.0);
                break;
            }
        }
    }
    public void retractSlide(double milliseconds){
        while(runtime.milliseconds() < milliseconds + 5000 && opModeIsActive()){
            if(slide.getCurrentPosition() <= 0 || slide.getCurrentPosition() < -2057){
                slide.setVelocity(15 * 28);
            }
            else{
                slide.setVelocity(0.0);
                break;
            }
        }
    }
    public void strafeRight(int deltaPosition, int initialPosition, double power){
        while(leftFrontDrive.getCurrentPosition() < initialPosition + deltaPosition && opModeIsActive()){
            //fr back, fl forward, br forward, bl back
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        }

        stopMotors();
    }
    public void strafeLeft(int deltaPosition, int initialPosition, double power){
        while(leftFrontDrive.getCurrentPosition() > initialPosition - deltaPosition && opModeIsActive()){
            //fr forward, fl back, br back, bl forward
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
        }
        stopMotors();
    }
    public void closeClaw(double milliseconds){
        clawServo.setPosition(0.0);
        wait(milliseconds, 1250);
    }
    public void openClaw(double milliseconds){
        clawServo.setPosition(1.0);
        wait(milliseconds, 1250);
    }
    public void centerClaw(double milliseconds){
        clawServo.setPosition(0.5);
        wait(milliseconds, 1000);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        double power = 0.5;

        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "br");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        clawServo = hardwareMap.get(Servo.class, "claw");





        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(runtime.milliseconds() < 30000 && opModeIsActive()){
//            turnLeft(0.5);
//            wait(runtime.milliseconds(), 1500);
//            turnRight(0.5);
            break;
        }
        /*
        close claw
        use an encoder to move forward
        then turn right 90 degrees
        then lift slide and move forward
        then open claw
        then back up and lower slide
        then strafe right to park
         */
    }


}
