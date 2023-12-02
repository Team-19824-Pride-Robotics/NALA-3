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
public class finalTest extends OpMode {

//pid
    private PIDController controller;
    public static double p = 0.005, i = 0, d =0;
    public static double f = 0;
    public static double target = 120;
    public static double pickup = 120;


    boolean liftControl = false;


    //lift
    DcMotorEx lift1;
    DcMotorEx lift2;


    public static double liftM = 20;
    public static double liftMax = 2000;

    //winch




    //arm

    double aPos =.01;

    public static double bPosx = .2;
    ServoImplEx Arm;
    ServoImplEx bucket;
    // AnalogInput sEncoder;
    AnalogInput sEncoder;
    AnalogInput sEncoder2;

    DcMotorEx intake;
    //drone





    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//pid
        controller = new PIDController(p,i,d);



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




    }

    @Override
    public void loop() {


        if (gamepad2.x) {
            intake.setPower(1);
        }
        if (gamepad2.b) {
            intake.setPower(-1);
        }
        if (gamepad2.a){
            intake.setPower(0);
        }

        target =  Range.clip(target, 110, liftMax);


        double pos = sEncoder.getVoltage() / 3.3 * 360;
        double pos2 = sEncoder2.getVoltage() / 3.3 * 360;

        aPos =  Range.clip(aPos, .01, .99);

        if (gamepad2.right_bumper){
            liftControl = true;
            target=300;
            bPosx=.55;
        }
        if (gamepad2.left_bumper){
            liftControl = false;
            bPosx=.2;
        }



        if (liftControl) {
            if (gamepad2.y){
                aPos = .99;
            }
            if (gamepad2.left_stick_y<.2 || gamepad2.left_stick_y>.2){
                target = gamepad2.left_stick_y * liftM + target;
            }
            if (gamepad1.start) {
                bPosx =.95;
            }
        }

        if (!liftControl) {
            if (pos2 >= 72 && pos2 <=76  ){
                aPos = .01;
            }
            if (pos >= 2 && pos<= 8) {
                target = pickup;
            }
        }


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

        telemetry.addData("target",target);
        telemetry.addData("Run time",getRuntime());
        telemetry.addData("1","test");
        telemetry.addData("pos1", lift1.getCurrentPosition());
        telemetry.addData("power1", lift1.getPower());
        telemetry.addData("pos2", lift2.getCurrentPosition());
        telemetry.addData("power2", lift2.getPower());
        telemetry.addData("arm", pos);
        telemetry.addData("bucket", pos2);


        telemetry.update();
    }

}