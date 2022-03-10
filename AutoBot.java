package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous

public class AutoBot extends LinearOpMode {
    //instanciranje robota
    ZvaneMk4 robot = new ZvaneMk4();
    //instanciranje vremena rada
    private ElapsedTime runtime = new ElapsedTime();
    //varijable za motore
    double snagaLijevogMotora;
    double snagaDesnogMotora;
    double gas;
    double skretanje;
    double gasDoDaske;
    //izvršavanje koda
    @Override
    public void runOpMode(){
        //podizanje sustava
        robot.init(hardwareMap);
        // šminka za publiku
        telemetry.addData("STATUS", "POKRETANJE");
        telemetry.update();
        robot.cekaj(1000);
        robot.resetirajGyro();
        telemetry.clearAll();
        telemetry.addData("STATUS", "RESETIRAM ŽIROSKOP");
        telemetry.update();
        robot.resetirajGyro();
        telemetry.clearAll();
        telemetry.addData("STATUS", "ŽIROSKOP RESET OK");
        telemetry.addData("Smjer: ", robot.vratiSmjer());
        telemetry.update();
        robot.cekaj(1000);
        telemetry.addData("STATUS", "PROVJERA SVIH SUSTAVA");
        telemetry.addData("Smjer: ", robot.vratiSmjer());
        telemetry.addData("Udaljenost: ", robot.senzorUdaljenosti.getDistance(DistanceUnit.CM));
        telemetry.addData("- : -----------------------------","-");
        telemetry.addData("Alpha. ", robot.lijeviSenzorBoje.alpha());
        telemetry.addData("Red: ", robot.lijeviSenzorBoje.red());
        telemetry.addData("Blue: ", robot.lijeviSenzorBoje.blue());
        telemetry.addData("Green: ", robot.lijeviSenzorBoje.green());
        telemetry.addData("Boja: ", robot.vratiBoju());
        telemetry.addData("- : -----------------------------","-");
        telemetry.addData("Snaga lijevog motora: ", robot.lijeviMotor.getPower());
        telemetry.addData("Snaga desnog motora: ", robot.lijeviMotor.getPower());
        telemetry.addData("Brzina ruke: ", robot.motorRuke.getVelocity());
        telemetry.addData("Pozicija ruke: ", robot.motorRuke.getCurrentPosition());
        telemetry.addData("Ruka na poziciji!", !robot.motorRuke.isBusy());
        telemetry.addData("- : -----------------------------","-");
        telemetry.addData("Hvataljka ", robot.servoHvataljke.getPosition());
        telemetry.addData("Touch ", robot.senzorDodira.isPressed());
        telemetry.update();

        waitForStart();

        runtime.reset();

        robot.lijeviMotor.setTargetPosition(1000);
        robot.desniMotor.setTargetPosition(1000);

        robot.lijeviMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.desniMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive())
        {

            ///                     ///
            ///                     ///
            /// OVDJE KREĆE MAGIJA  ///
            ///                     ///
            ///                     ///

            // početak igre



            // Telemetrija robota
            telemetry.addData("STATUS", "SVE RADI");
            telemetry.addData("Kut: ", robot.vratiKut());
            telemetry.addData("Pozicija lijevog motora: ", robot.lijeviMotor.getCurrentPosition());
            telemetry.addData("Pozicija desnog motora: ", robot.desniMotor.getCurrentPosition());
            telemetry.addData("Boja: ", robot.vratiBoju());
            telemetry.update();
        }

    }
}
