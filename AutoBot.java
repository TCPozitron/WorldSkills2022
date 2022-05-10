package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;


@Autonomous

public class AutoBot extends LinearOpMode {

    //instanciranje robota
    ZvaneMk4 robot = new ZvaneMk4();
    //instanciranje vremena rada
    private ElapsedTime runtime = new ElapsedTime();

    int kut=90;
    int brzina = 1500;
    double snaga = 0.3;
    double ir0s;
    double ir1s;
    double ir2s;
    double ir3s;
    double ir4s;
    double maxV = 3.3;
    double granicaLinijeIR = 0.7;
    double granicaLinijeAlpha = 600;

    double PID = 0;
    double P = 0;
    double I = 0;
    double D = 0;
    double Kp = 0.20;
    double Ki = 0.01;
    double Kd = 0.01;
    double greskaLinije  = 0;
    double prethodnaGreska = 0;
    double korekcija = 0;

    public double traziLiniju(){
        ir0s = 1 - robot.IR0.getVoltage() / maxV;
        ir1s = 1 - robot.IR1.getVoltage() / maxV;
        ir2s = robot.lijeviSenzorBoje.alpha();
        ir3s = 1 - robot.IR2.getVoltage() / maxV;
        ir4s = 1 - robot.IR3.getVoltage() / maxV;


        if (ir0s > granicaLinijeIR)
            ir0s = 0;
        else
            ir0s = 1;

        if (ir1s > granicaLinijeIR)
            ir1s = 0;
        else
            ir1s = 1;

        if (ir2s > granicaLinijeAlpha)
            ir2s = 0;
        else
            ir2s = 1;

        if (ir3s > granicaLinijeIR)
            ir3s = 0;
        else
            ir3s = 1;

        if (ir4s > granicaLinijeIR)
            ir4s = 0;
        else
            ir4s = 1;

        //-------------------------------------------------------------
        /* podaci sa senzora za praćenje linije
        Polje senzora 	    Greška
        0 0 0 0 1	        4
        0 0 0 1 1           3
        0 0 0 1 0	        2
        0 0 1 1 0	        1
        0 0 1 0 0	        0
        0 1 1 1 0           0
        0 1 1 0 0	       -1
        0 1 0 0 0	       -2
        1 1 0 0 0	       -3
        1 0 0 0 0	       -4

        1 1 1 1 1           9 neprekidna linija
        0 0 0 0 0           8 nema linije
        1 1 1 0 0           7 Skretanje lijevo 90°
        1 1 1 1 0           7 Skretanje lijevo 90°
        0 0 1 1 1           6 Skretanje desno 90°
        0 1 1 1 1           6 Skretanje desno 90°
        */

        if (ir0s == 0 && ir1s == 0 && ir2s == 0 && ir3s == 0 && ir4s == 1)
            greskaLinije = 4;
        if (ir0s == 0 && ir1s == 0 && ir2s == 0 && ir3s == 1 && ir4s == 1)
            greskaLinije = 3;
        if (ir0s == 0 && ir1s == 0 && ir2s == 0 && ir3s == 1 && ir4s == 0)
            greskaLinije = 2;
        if (ir0s == 0 && ir1s == 0 && ir2s == 1 && ir3s == 1 && ir4s == 0)
            greskaLinije = 1;
        if (ir0s == 0 && ir1s == 0 && ir2s == 1 && ir3s == 0 && ir4s == 0)
            greskaLinije = 0;
        if (ir0s == 0 && ir1s == 1 && ir2s == 1 && ir3s == 1 && ir4s == 0)
            greskaLinije = 0;
        if (ir0s == 0 && ir1s == 1 && ir2s == 1 && ir3s == 0 && ir4s == 0)
            greskaLinije = -1;
        if (ir0s == 0 && ir1s == 1 && ir2s == 0 && ir3s == 0 && ir4s == 0)
            greskaLinije = -2;
        if (ir0s == 1 && ir1s == 1 && ir2s == 0 && ir3s == 0 && ir4s == 0)
            greskaLinije = -3;
        if (ir0s == 1 && ir1s == 0 && ir2s == 1 && ir3s == 0 && ir4s == 0)
            greskaLinije = -4;
        if (ir0s == 1 && ir1s == 1 && ir2s == 1 && ir3s == 1 && ir4s == 1)
            greskaLinije = 9;
        if (ir0s == 0 && ir1s == 0 && ir2s == 0 && ir3s == 0 && ir4s == 0)
            greskaLinije = 8;
        if (ir0s == 1 && ir1s == 1 && ir2s == 1 && ir3s == 0 && ir4s == 0)
            greskaLinije = 7;
        if (ir0s == 1 && ir1s == 1 && ir2s == 1 && ir3s == 1 && ir4s == 0)
            greskaLinije = 7;
        if (ir0s == 0 && ir1s == 0 && ir2s == 1 && ir3s == 1 && ir4s == 1)
            greskaLinije = 6;
        if (ir0s == 0 && ir1s == 1 && ir2s == 1 && ir3s == 1 && ir4s == 1)
            greskaLinije = 6;

        return greskaLinije;
    }

    double racunajPID(){
        greskaLinije = traziLiniju();
        if (greskaLinije<5) {
            P = greskaLinije;
            I = I + greskaLinije;
            D = greskaLinije - prethodnaGreska;
            PID = (Kp * P) + (Ki * I) + (Kd * D);
            prethodnaGreska = greskaLinije;
        }
        else if (greskaLinije == 6){
            PID = greskaLinije;
        }
        else if (greskaLinije == 7){
            PID = greskaLinije;
        }
        else if (greskaLinije == 8){
            PID = greskaLinije;
        }
        else if (greskaLinije == 9){
            PID = greskaLinije;
        }
        return PID;
    }

    void pratiLinijuPID(){
        korekcija = racunajPID();
        if (korekcija<5){
            double snagaMaxD = snaga+korekcija;
            if (snagaMaxD > 1)
                snagaMaxD = 1;
            double snagaMaxL = snaga-korekcija;
            if (snagaMaxL > 1)
                snagaMaxL = 1;
            robot.desniMotor.setPower(snagaMaxD);
            robot.desniMotor.setPower(snagaMaxL);
        }
        else if (korekcija == 6){
            while (robot.vratiKut() > -kut) {
                robot.desniMotor.setPower(snaga/2);
                robot.desniMotor.setPower(-snaga/2);
            }
        }
        else if (korekcija == 7){
            while (robot.vratiKut() < kut) {
                robot.desniMotor.setPower(-snaga/2);
                robot.desniMotor.setPower(snaga/2);
            }
        }
        else if (korekcija == 8){
            //što raditi kada nema linije?
        }
        else if (korekcija == 9){
            //kada je puna linija treba ići malo naprijed pa
            // ponovo provjeriti šta je poslije te linije
            robot.stani();
            int pozicijaD = robot.desniMotor.getCurrentPosition();
            int pozicijaL = robot.desniMotor.getCurrentPosition();
            robot.desniMotor.setTargetPosition(pozicijaD+5);
            robot.lijeviMotor.setTargetPosition(pozicijaL+5);
            robot.desniMotor.setVelocity(brzina);
            robot.lijeviMotor.setVelocity(brzina);
            // i šta sad?
        }

    }

    double[] detekcijaPrepreke(){
        double[] podaci = {0,0};
        double udaljenost = 100;
        robot.rezervniServo.setPosition(0);
        for (double i = 0; i<=100; i=i+25) {
            double j = i / 100;
            robot.rezervniServo.setPosition(j);
            udaljenost = robot.senzorUdaljenosti.getDistance(DistanceUnit.CM);
            podaci[0] = udaljenost;
            podaci[1] = robot.rezervniServo.getPosition();
            return podaci;
        }
        for (double i = 100; i<=0; i=i-25) {
            double j = i / 100;
            robot.rezervniServo.setPosition(j);
            udaljenost = robot.senzorUdaljenosti.getDistance(DistanceUnit.CM);
            podaci[0] = udaljenost;
            podaci[1] = robot.rezervniServo.getPosition();
            return podaci;
        }

        return podaci;
    }




    //izvršavanje koda
    @Override
    public void runOpMode(){
        //podizanje sustava
        robot.init(hardwareMap);

        telemetry.clearAll();
        telemetry.addData("STATUS", "RESETIRAM ŽIROSKOP");
        telemetry.addData("Smjer: ", robot.vratiSmjer());
        telemetry.update();
        robot.resetirajGyro();
        robot.cekaj(500);
        telemetry.clearAll();
        telemetry.addData("STATUS", "ČEKAM START");
        telemetry.addData("Smjer: ", robot.vratiSmjer());
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {

            pratiLinijuPID();
            while (opModeIsActive() && robot.vratiKodBojeL() != 2) {
                telemetry.clearAll();
                telemetry.addData("STATUS", "TEST IR SENZORA");
                telemetry.addData("IR0: ", ir0s);
                telemetry.addData("IR1: ", ir1s);
                telemetry.addData("IR2: ", ir2s);
                telemetry.addData("IR3: ", ir3s);
                telemetry.addData("IR4: ", ir4s);
                telemetry.addData("- : -----------------------------", "-");
                telemetry.addData("Kod boje D: ", robot.vratiKodBojeD());
                telemetry.update();
            }
            robot.stani();
            robot.cekaj(100);
            robot.voziRavno(snaga);
            while(opModeIsActive() && detekcijaPrepreke()[0] > 25){
                telemetry.clearAll();
                telemetry.addData("STATUS", "Vozim ravno do prepreke");
                telemetry.addData("Smjer: ", robot.vratiSmjer());
                telemetry.addData("Udaljenost cm: ", detekcijaPrepreke()[0]);
                telemetry.update();
            }
            robot.cekaj(5000);



        }


    }
}
