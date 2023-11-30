package org.firstinspires.ftc.teamcode.teleop;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(group = "teleop")
public class teleop extends OpMode {


    boolean liftControl = false;


    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;
    double rotate;
    //intake
    DcMotorEx intake;
    public static double power = 1;
    public static double backPower =-1;
    //lift
    DcMotorEx lift1;
    DcMotorEx lift2;
    public static double liftPower = 1;
    public static double liftPos = 0;
    public static double liftM = 10;
    public static double liftMax = 2000;

    //winch
    DcMotorEx winch;

    public static int wBack = -200;
    public static int wUp = 2000;
    public static int wDown = 1500;
    public static double wbPower = .5;
    public static double wuPower = 1;
    public static double wdPower = 1;



    //arm

    double aVar;
    public static double bVar = 64;
    public static double bPosx = .18;
    double aPos;
    ServoImplEx Arm;
    ServoImplEx bucket;
    // AnalogInput sEncoder;
    AnalogInput sEncoder;
    AnalogInput sEncoder2;

    //drone
    Servo drone;
    public static double launch = .5;




    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        FR.setDirection(DcMotorEx.Direction.REVERSE);

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);


        Arm= (ServoImplEx) hardwareMap.get(Servo.class, "lArm");
        bucket = (ServoImplEx) hardwareMap.get(Servo.class, "bucket");
        Arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sEncoder = hardwareMap.get(AnalogInput.class, "sEncoder");
        sEncoder2 = hardwareMap.get(AnalogInput.class, "sEncoder2");


        intake = hardwareMap.get(DcMotorEx.class, "intake");

        drone = hardwareMap.get(Servo.class, "drone");

        winch = hardwareMap.get(DcMotorEx.class, "winch");



    }

    @Override
    public void loop() {
        telemetry.addData("Run time",getRuntime());
        telemetry.addData("FR Power", FR.getPower());
        telemetry.addData("BR Power",BR.getPower());
        telemetry.addData("FL Power", FL.getPower());
        telemetry.addData("BL Power",BL.getPower());
        telemetry.addData("1","test");
        telemetry.addData("pos1", lift1.getCurrentPosition());
        telemetry.addData("power1", lift1.getPower());
        telemetry.addData("pos2", lift2.getCurrentPosition());
        telemetry.addData("power2", lift2.getPower());
        telemetry.update();
        liftPos =  Range.clip(liftPos, 110, liftMax);

        double d_power = .8-.4*gamepad1.left_trigger+(.5*gamepad1.right_trigger);
        double drive = gamepad1.left_stick_y;
        double rotate_stick = -gamepad1.right_stick_x;
        double rotate_button = 0;

        rotate = rotate_stick + .5*rotate_button;

        BL.setPower(drive + rotate);
        FL.setPower(drive + rotate);
        BR.setPower(drive - rotate);
        FR.setPower(drive - rotate);

        if (gamepad1.dpad_up) {
            BL.setPower(-d_power);
            FL.setPower(-d_power);
            BR.setPower(-d_power);
            FR.setPower(-d_power);
        }
        else if (gamepad1.dpad_down) {
            BL.setPower(d_power);
            FL.setPower(d_power);
            BR.setPower(d_power);
            FR.setPower(d_power);
        }
        else if (gamepad1.dpad_left) {
            BL.setPower(-d_power);
            FL.setPower(d_power);
            BR.setPower(d_power);
            FR.setPower(-d_power);
        }
        else if (gamepad1.dpad_right) {
            BL.setPower(d_power);
            FL.setPower(-d_power);
            BR.setPower(-d_power);
            FR.setPower(d_power);
        }

        //drone
        if (gamepad1.b){
            drone.setPosition(launch);
        }
        //winch
        if (gamepad1.x){
            lift1.setTargetPosition(wBack);
            lift2.setPower(wbPower);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.y){
            lift1.setTargetPosition(wUp);
            lift2.setPower(wuPower);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.a){
            lift1.setTargetPosition(wDown);
            lift2.setPower(wdPower);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //DIVER 2//

//intake
        if (gamepad2.x) {
            intake.setPower(power);
        }
        if (gamepad2.b) {
            intake.setPower(backPower);
        }
        if (gamepad2.a){
            intake.setPower(0);
        }

        double pos = sEncoder.getVoltage() / 3.3 * 360;
        double pos2 = sEncoder2.getVoltage() / 3.3 * 360;

        aPos =  Range.clip(aPos, .15, .99);
        aVar =  Range.clip(aVar, 53, 355);

        bVar =  Range.clip(bVar, 64, 225);
        bPosx =  Range.clip(bPosx, .15, .6);

        if (gamepad2. right_bumper){
            liftControl = true;
            liftPos=300;
        }
        if (gamepad2.left_bumper){
            liftControl = false;
            bVar=.5;
        }



        if (liftControl) {
            if (gamepad2.left_stick_y<.2 || gamepad2.left_stick_y>.2){
                liftPos= gamepad2.left_stick_y * liftM + liftPos;
            }
            if (gamepad2.right_stick_y < .2 || gamepad2.right_stick_y > -.2) {
                aVar = aVar + gamepad2.right_stick_y *10;
            }
            if (gamepad2.left_stick_y < .2 || gamepad2.left_stick_y > -.2) {
                bVar = bVar + gamepad2.left_stick_y;
            }
        }

        if (!liftControl) {
            if (pos2 <= 200 || pos2 >=190  ){
                aVar = .5;
            }
            if (pos <= 10 || pos>= 0) {
                liftPos = 110;
            }
        }

        aPos = aVar * .0028;//tf is this
        bPosx= bVar *.0028;

        Arm.setPosition(aPos);
        bucket.setPosition(bPosx);

        lift1.setTargetPosition((int) liftPos);
        lift2.setPower(liftPower);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        lift2.setTargetPosition((int) liftPos);
        lift2.setPower(liftPower);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}