package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="TeleTest", group="TeleOp")

public class TeleTest extends LinearOpMode {
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

        telemetry.addData("Kut: ", robot.vratiKut());
        telemetry.addData("Pozicija lijevog motora: ", robot.lijeviMotor.getCurrentPosition());
        telemetry.addData("Pozicija desnog motora: ", robot.desniMotor.getCurrentPosition());
        telemetry.update();
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


        while (opModeIsActive())
        {
            // postavljanje gamepad (a, b, x, y)
            boolean g1trokut = gamepad1.a;
            boolean g1X = gamepad1.y;
            boolean g1kvadrat = gamepad1.b;
            boolean g1krug = gamepad1.x;

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            gas = -gamepad1.left_stick_y * robot.MAX_SNAGA_MOTORA;
            skretanje  =  gamepad1.right_stick_x * robot.MAX_SNAGA_MOTORA;

            // kombiniranje pogona
            snagaLijevogMotora  = gas + skretanje;
            snagaDesnogMotora = gas - skretanje;

            // normalizacija pogona na 100% snage +/- 1.0
            gasDoDaske = Math.max(Math.abs(snagaLijevogMotora), Math.abs(snagaDesnogMotora));
            if (gasDoDaske > 1.0)
            {
                snagaLijevogMotora /= gasDoDaske;
                snagaDesnogMotora /= gasDoDaske;
            }

            // kretanje motora
            robot.vozi(snagaLijevogMotora, snagaDesnogMotora);

            // kontrola robota pomoću gamepada

            if (g1X)
                robot.pomakniRuku(290, 200);
            if (g1trokut)
                robot.pomakniRuku(0, 300);
            if (g1kvadrat)
                robot.servoHvataljke.setPosition(1.0);
            if (g1krug)
                robot.servoHvataljke.setPosition(0.0);
            if (gamepad1.left_bumper)
                robot.pratiLiniju();
            if (gamepad1.right_bumper)
                robot.stani();
            if (gamepad1.dpad_up)
                robot.servoProduzetakRuke.setPosition(1.0);
            if (gamepad1.dpad_down)
                robot.servoProduzetakRuke.setPosition(0.0);

            // Telemetrija robota
            telemetry.addData("STATUS", "SVE RADI");
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
            telemetry.addData("Korekcija po boji: ", robot.korekcijaPoBoji);
            telemetry.addData("Pozicija ruke: ", robot.motorRuke.getCurrentPosition());
            telemetry.addData("Ruka na poziciji!", !robot.motorRuke.isBusy());
            telemetry.addData("- : -----------------------------","-");
            telemetry.addData("Hvataljka ", robot.statusHvataljke());
            telemetry.addData("Touch ", robot.senzorDodira.isPressed());
            telemetry.update();
        }
    }
}
