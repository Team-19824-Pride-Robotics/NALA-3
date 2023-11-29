package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(group = "teleop")
@Config
public class liftTest extends OpMode {




    boolean liftControl = false;
    public static int one = 0;
    public static int two = 1000;
    public static int three  = 500;
    public static int lIn  = 110;
    public static int lOut  = 200;

    public static double liftPower = 1;



    private DcMotorEx lift1;
    private DcMotorEx lift2;



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);



    }

    @Override
    public void loop() {

        if (gamepad2.right_bumper){
            liftControl = true;
            lift1.setTargetPosition(lOut);
            lift2.setPower(liftPower);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift2.setTargetPosition(lOut);
            lift2.setPower(liftPower);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.left_bumper){
            liftControl = false;
        }

        if (liftControl) {
            if (gamepad2.y) {
                lift1.setTargetPosition(one);
                lift2.setPower(liftPower);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                lift2.setTargetPosition(one);
                lift2.setPower(liftPower);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (gamepad2.x) {
                lift1.setTargetPosition(two);
                lift2.setPower(liftPower);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                lift2.setTargetPosition(two);
                lift2.setPower(liftPower);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.a) {
                lift1.setTargetPosition(three);
                lift2.setPower(liftPower);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                lift2.setTargetPosition(three);
                lift2.setPower(liftPower);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        }
        else {
            lift1.setTargetPosition(lIn);
            lift2.setPower(liftPower);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            lift2.setTargetPosition(lIn);
            lift2.setPower(liftPower);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        telemetry.addData("pos1", lift1.getCurrentPosition());
        telemetry.addData("power1", lift1.getPower());

        telemetry.addData("pos2", lift2.getCurrentPosition());
        telemetry.addData("power2", lift2.getPower());
        telemetry.update();
    }


}