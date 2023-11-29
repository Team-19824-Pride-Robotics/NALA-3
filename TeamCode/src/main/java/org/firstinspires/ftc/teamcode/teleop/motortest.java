package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(group = "teleop")
@Config
public class motortest extends OpMode {



    public static int one = -2000;
    public static int two = 300;
    public static int three  = -800;

    public static double power = 1;



    private DcMotorEx m1;



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        m1 = hardwareMap.get(DcMotorEx.class, "winch");
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    @Override
    public void loop() {

        if (gamepad1.y) {
            m1.setTargetPosition(one);
            m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m1.setPower(power);
        }
        if (gamepad1.x) {
            m1.setTargetPosition(two);
            m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m1.setPower(power);
        }
        if (gamepad1.a) {
            m1.setTargetPosition(three);
            m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m1.setPower(power);
        }


        telemetry.addData("pos", m1.getCurrentPosition());
        telemetry.addData("power", m1.getPower());
        telemetry.update();
    }


}