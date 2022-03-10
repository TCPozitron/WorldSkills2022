package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ZvaneMk4
{
    // Inicijalizacija aktuatora
    public DcMotorEx lijeviMotor;
    public DcMotorEx desniMotor;
    public DcMotorEx motorRuke;
    public Servo rezervniServo;
    public Servo servoProduzetakRuke;
    public Servo servoHvataljke;
    //inicijalizaija senzora
    public ColorSensor lijeviSenzorBoje;
    public ColorSensor desniSenzorboje;
    public DistanceSensor senzorUdaljenosti;
    public BNO055IMU gyro;
    public TouchSensor senzorDodira;
    public Orientation orijentacija = new Orientation();
    // globalne varijable
    public double kutKretanja, korekcijaPoBoji, korekcijaPoSmjeru;
    public final double VRIJEDNOST_RUBA_LINIJE = 0.300;
    // maksimalna snaga motora
    public double MAX_SNAGA_MOTORA = 0.5;
    //inicijalizacija postavki hardvera
    HardwareMap mapaDijelova;


//definiranje hardvera
    public void init(HardwareMap mapaDijelova) {

        // Definiranje aktuatora
        lijeviMotor =mapaDijelova.get(DcMotorEx.class, "lijeviMotor");
        desniMotor =mapaDijelova.get(DcMotorEx.class, "desniMotor");
        motorRuke =mapaDijelova.get(DcMotorEx.class, "motorRuke");
        rezervniServo =mapaDijelova.get(Servo.class, "rezervniServo");
        servoProduzetakRuke =mapaDijelova.get(Servo.class, "servoProduzetakRuke");
        servoHvataljke=mapaDijelova.get(Servo.class, "servoHvataljke");
        // definiranje senzora
        lijeviSenzorBoje =mapaDijelova.get(ColorSensor.class, "lijeviSenzorboje");
        //desniSenzorBoje=mapaDijelova.get(ColorSensor.class, "desniSenzorBoje");
        senzorUdaljenosti =mapaDijelova.get(DistanceSensor.class, "senzorUdaljenosti");
        gyro = mapaDijelova.get(BNO055IMU.class, "gyro");
        senzorDodira = mapaDijelova.get(TouchSensor.class, "dodir");

        // Isključivanje motora
        lijeviMotor.setPower(0);
        desniMotor.setPower(0);
        motorRuke.setPower(0);

        //Definiranje korištenja enkodera
        //bez enkodera
        lijeviMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        desniMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorRuke.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // sa enkoderom
        motorRuke.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRuke.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            /*
            lijeviMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            desniMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lijeviMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            desniMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            */

        //Definicija ponašanja motora kod 0% snage
        lijeviMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        desniMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRuke.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Okretanje motora u pravom smjeru (ovisi o orijentaciji motora)
        lijeviMotor.setDirection(DcMotor.Direction.REVERSE);
        //desniMotor.setDirection(DcMotor.Direction.REVERSE);

        //kalibracija žiroskopa
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        gyro.initialize(parameters);
    }
//kretanje robota
    public void vozi(double lijeviMotor, double desniMotor){
        this.lijeviMotor.setPower(lijeviMotor);
        this.desniMotor.setPower(desniMotor);
    }
    public void okreniSeLijevo(int kut, double snaga)
    {
        if(vratiKut()<kut) {
            lijeviMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            desniMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            lijeviMotor.setPower(-snaga);
            desniMotor.setPower(snaga);

        }
        else if(vratiKut()>=kut){
            lijeviMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            desniMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            stani();
        }
    }
    public void okreniSeDesno(int kut, double snaga)
    {

        if(vratiKut()>-kut) {
            lijeviMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            desniMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            lijeviMotor.setPower(snaga);
            desniMotor.setPower(-snaga);

        }
        else if(vratiKut()<=-kut){
            lijeviMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            desniMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            stani();
        }
    }
    //pomicanje ruke [-1,1] gore ili dole pomoću enkodera
    public void pomakniRuku(int pozicija, double brzina){
        motorRuke.setTargetPosition(pozicija);
        motorRuke.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRuke.setVelocity(brzina);
    }
    // dohvaćanje statusa hvataljke
    public String statusHvataljke()
    {
        String statusHvataljke = "";
        if (servoHvataljke.getPosition() == 0)
            statusHvataljke = "Zatvorena";
        else
            statusHvataljke = "Otvorena";

        return statusHvataljke;
    }
    public void stani(){
        lijeviMotor.setPower(0);
        desniMotor.setPower(0);
    }
//praćenje linije
    // treba popraviti jer ne prati liniju ako treba skrenuti za 90°
    public void pratiLiniju(){
        double svjetlost = lijeviSenzorBoje.alpha();
        korekcijaPoBoji = VRIJEDNOST_RUBA_LINIJE - (svjetlost / 1000);
        if (korekcijaPoBoji <= 0){
            lijeviMotor.setPower(MAX_SNAGA_MOTORA + korekcijaPoBoji);
            desniMotor.setPower(MAX_SNAGA_MOTORA);
        } else {
            lijeviMotor.setPower(MAX_SNAGA_MOTORA);
            desniMotor.setPower(MAX_SNAGA_MOTORA - korekcijaPoBoji);
        }
    }
    //promjena kriterija za skretanje
    public void pratiLiniju2(){
        if(lijeviSenzorBoje.alpha()>340){
            desniMotor.setPower(MAX_SNAGA_MOTORA+0.1);
            lijeviMotor.setPower(MAX_SNAGA_MOTORA);
        }
        else if(lijeviSenzorBoje.alpha()<340){
            desniMotor.setPower(MAX_SNAGA_MOTORA);
            lijeviMotor.setPower(MAX_SNAGA_MOTORA + 0.1);
        }
        else{
            desniMotor.setPower(MAX_SNAGA_MOTORA);
            lijeviMotor.setPower(MAX_SNAGA_MOTORA);
        }
    }
    //dodavanje drugog senzora za boju
    public void pratiLiniju3(){
        double svjetlost = lijeviSenzorBoje.alpha();
        svjetlost = svjetlost - VRIJEDNOST_RUBA_LINIJE;
    }
//pauza
    public void cekaj(int vrijeme){
        try {
            Thread.sleep(vrijeme);

        } catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
//resetiranje žiroskopa
    //postavljanje početne orjentacije z-osi na 0°
    public void resetirajGyro(){

        orijentacija = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        kutKretanja = 0;
    }
    //Dohvaćanje kuta Z osi robota (otklon od početnog položaja)
    public double vratiKut()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - orijentacija.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        kutKretanja += deltaAngle;

        orijentacija = angles;

        return kutKretanja;
    }
    //dohvaćanje korekcije smjera
    public double vratiSmjer()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = vratiKut();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
//definiranje kriterija za prepoznavanje boje
    //dohvaćanje naziva boje
    public String vratiBoju()
    {
        int alpha = lijeviSenzorBoje.alpha();
        int crvena = lijeviSenzorBoje.red();
        int plava = lijeviSenzorBoje.blue();
        int zelena = lijeviSenzorBoje.green();
        String boja = "Nepoznato";
        if (alpha > 450)
            boja = "Bijela";
        else if (alpha>200 && alpha < 260 && crvena < 185 && plava < 280 && zelena<310)
            boja = "Crna";
        else if (alpha < 160 && zelena < crvena && plava < zelena)
            boja = "Crvena";
        else if (crvena > plava && zelena > crvena)
            boja = "Žuta";
        else if (zelena>crvena && zelena > plava)
            boja = "Zelena";
        else if (zelena > crvena && zelena < plava)
            boja = "Plava";

        return boja;
    }
}