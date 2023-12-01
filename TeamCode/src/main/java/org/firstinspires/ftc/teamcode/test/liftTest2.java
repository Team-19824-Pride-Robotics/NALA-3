package org.firstinspires.ftc.teamcode.test;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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
@TeleOp(group = "test")
public class liftTest2 extends OpMode {

//pid
    private PIDController controller;
    public static double p = 0.005, i = 0, d =0;
    public static double f = 0;
    public static double target = 110;


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

    double aVar =354;
    double aPos =.99;

    public static double bVar = 214;
    public static double bPosx = .6;
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
//pid
        controller = new PIDController(p,i,d);

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


        Arm= (ServoImplEx) hardwareMap.get(Servo.class, "Arm");
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


        telemetry.addData("target",target);
        telemetry.addData("Run time",getRuntime());
        telemetry.addData("1","test");
        telemetry.addData("pos1", lift1.getCurrentPosition());
        telemetry.addData("power1", lift1.getPower());
        telemetry.addData("pos2", lift2.getCurrentPosition());
        telemetry.addData("power2", lift2.getPower());
        telemetry.update();
        target =  Range.clip(target, 110, 2400);

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
            winch.setTargetPosition(wBack);
            winch.setPower(wbPower);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.y){
            winch.setTargetPosition(wUp);
            winch.setPower(wuPower);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.a){
            winch.setTargetPosition(wDown);
            winch.setPower(wdPower);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        aPos =  Range.clip(aPos, .01, .99);
        aVar =  Range.clip(aVar, 5, 354);

        if (gamepad2.right_bumper){
            liftControl = true;
            target=300;
        }
        if (gamepad2.left_bumper){
            liftControl = false;
            bPosx=.6;
        }



        if (liftControl) {
            if (gamepad2.left_stick_y<.2 || gamepad2.left_stick_y>.2){
                target = gamepad2.left_stick_y * liftM + target;
            }
            if (gamepad2.right_stick_y < .2 || gamepad2.right_stick_y > -.2) {
                aVar = aVar + gamepad2.right_stick_y *10;
            }
            if (gamepad1.start) {
                bPosx =.4;
            }
        }

        if (!liftControl) {
            if (pos2 >= 140 && pos2 <=145  ){
                aVar = 354;
            }
            if (pos >= 352 && pos<= 354) {
                target = 110;
            }
        }

        aPos = aVar * .0028;//tf is this

        controller.setPID(p, i, d);
        int liftPos1 = lift1.getCurrentPosition();
        int liftPos2 = lift2.getCurrentPosition();
        double pid = controller.calculate(liftPos1, target);
        double pid2 = controller.calculate(liftPos2, target);
        double ff = 0;

        double lPower1 = pid +ff;
        double lPower2 = pid2 +ff;

        lift1.setPower(lPower1);
        lift2.setPower(lPower2);
        Arm.setPosition(aPos);
        bucket.setPosition(bPosx);

    }

}