/*
                        |-----------------|
                        | Lineal Velocity |---------|
                        |-----------------|    |    |
                                               |    |
3500-----|            |-------------|          |   +|                  |-----------------|     |--------------|
          +|---|      |             |          |   |---|    +|---|     |                 |     |              |
-----------|   |------|  PID ANGLE  |----- --------|   |-----|   |-----| PI VELOCITY LEFT|-----|  LEFT MOTOR  |-----------------------
          -|---|      |             |      |   |  -|---|    -|---|     |             |   |     |              |            |          |
             |        |-------------|      |   |               |       |-----------------|     |--------------|            |         +|
             |                             |   |               |                                                           |        |----|
             |                             |   |               |                                                           |        |    |-----
             |                             |   |                -----------------------------------------------------------         |----|      |
             |                             |   |                                                                                      +|        |
             |                             |   |                                                                                       |        |
             |                             |   |   +                   |-------------------|     |---------------|                     |        |
             |                             |   |---|---|    +|---|     |                   |     |               |                     |        |
             |                             |       |   |-----|   |-----| PI VELOCITY RIGTH |-----|  RIGTH MOTOR  |---------------------         |
             |                             |-------|---|    -|---|     |                   |     |               |           |                  |
             |                                    +            |       |-------------------|     |---------------|            |                 |
             |                                                 |                                                              |                 |
             |                                                 |                                                              |                 |
             |                                                 ---------------------------------------------------------------                  |
             |                                                                                                                                  |
             |                                                                                                                                  |
             |                                                                                                                                  |
             |----------------------------------------------------------------------------------------------------------------------------------
*/ /***************************************************************
    UNIVERSIDAD DE IBAGUÉ  / D+TEC / SIRUI

    This file content the control of a Line Follower equiped with an engine and 
    a master/slave architecture control. The system is based on a MBED micro,
    8QTRA pololu line sensors, IMU and encoders in a diferential topology.

********************************************************************************/
#include "math.h"
#include "QEI.h"
#include "mbed.h"
#include <stdio.h>
#include "Servo.h"
//------ Definicion de pines I/O

DigitalOut myled(LED1);         // LED del ARM
DigitalOut LED_Iz(p26);         // LED1
DigitalOut LED_Der(p18);        // LED2
DigitalIn  Button_1(p7);        // Button_1
DigitalIn  Button_2(p8);        // Button_2
DigitalIn  IR(p9);        // RCIR
//********************//

//------Salidas PWM

PwmOut MI1(p24);                // Lef Motor 1
PwmOut MI(p23);                 // Lef Motor 2
PwmOut MD1(p22);                // Rigth Motor 1
PwmOut MD(p21);                 // Rigth Motor 2
Servo      Turbina(p25);
//********************//

//---------------ENCODERS--------------//
//---- Objeto encoder
QEI wheel_R(p16, p15, NC, 1);     // Rigth encoder
QEI wheel_L(p5, p6, NC, 1);      // Rigth encoder

//-------Variables encoder---//
float difTime_L,difTime_R;
float dW_L,w_L,dW_R,w_R;
long oldPosition_L  =0.0;
long oldPosition_R  =0.0;
int tic_L,toc_L, tic_R,toc_R;
long newPosition_R, newPosition_L;
int lastPosition = 0.0;

//*****Encoder signal filter
const float filt_coef_B[2]={0.112160244475193, 0.112160244475193};
const float filt_coef_A[1]={ -0.775679511049613};
float W_L[3]={0,0,0};
float W_L_filtered[3]={0,0,0};
float W_R[3]={0,0,0};
float W_R_filtered[3]={0,0,0};
//*****************END ENCODERS*********************//

//---- Comunicacion Serial
Serial pc(USBTX, USBRX);        // USB connection
Serial BLE(p13,p14);            // tx, rx Bluethoot serial
Serial blue(p28,p27);           // UART0_TX, UART0_RX / slave micro-controller
//********************//

//----- Measure and sample time.
Timer T;
long start,finish;
long Ts = 8000.0;
//********************//


//****************CONTROLLERS********//

//----Angel Coontrol-------//
float LineError=0.0;
float LineError2=0.0;
int line_position;
float Kp_Line,Kd_Line;
float Rads_Max=2*350,Rads_Min=-2*350;
float U_Line;
float Ref;
//**********//

//----Velocity Control----//
float  Error_WL,Error_WL2,Kd_L,Kp_L,Ki_L,U_wl,U_wl2,U_iL,U_ikL;
float  Error_WR,Error_WR2,Kd_R,Kp_R,Ki_R,U_wr,U_wr2,U_iR,U_ikR;
float  Ref_VelWL,Ref_VelWR;
float UR,UL;
float AAA;
float Velocidad, V_MAX;
//************END CONTROL*********//






int muestras=0;
int Lecturas[10]; //Variables "filtro"
int i = 0;float Total = 0;float Promedio = 0;
int h = 0;
int Clase; // 1, Blanco,2Negro,3 en linea
int Slave_Data;
#define size_buffer 256
char buffer[size_buffer];
uint16_t count_data;

//------------- Functions ---------------------//
int samples=0;
int Linea(){
    for(int i=0;i<size_buffer;i++){
        buffer[i] = NULL;
    }
    count_data = 0;
    blue.putc(1);
    int i;
    if(blue.readable()>0){
    char buffer [size_buffer];
    blue.gets (buffer, size_buffer);
    i = atoi (buffer);
    }
    return i;
    }

void Calibration(){
     //*** Buttom 2 for Caliration***///
    pc.printf(" Entrada Linea ");
    while(1){
        h=Linea();
        LED_Der  = 1,LED_Iz  = 0;
        wait_ms(350);
        LED_Der  = 0,LED_Iz  = 1;
        wait_ms(300);
        LED_Der  = 1,LED_Iz  = 0;
        wait_ms(250);
        LED_Der  = 0,LED_Iz  = 1;
        wait_ms(200);
        LED_Der  = 1,LED_Iz  = 1;
        if(h==3){
            pc.printf(" Calibrado");
            LED_Iz  = 0,LED_Der  = 1;
            wait_ms(350);
            LED_Der  = 0,LED_Iz  = 1;
            wait_ms(350);
            LED_Iz  = 0,LED_Der  = 1;
            wait_ms(350);
            LED_Der  = 0,LED_Iz  = 1;
            wait_ms(350);
            LED_Der  = 0,LED_Iz  = 0;
            wait_ms(550);
            LED_Der  = 1,LED_Iz  = 1;
            wait_ms(750);
            LED_Der  = 0,LED_Iz  = 0;
            wait_ms(750);
            LED_Der  = 1,LED_Iz  = 1;
            wait_ms(750);
            LED_Der  = 0,LED_Iz  = 0;
            wait_ms(750);
            break;
        } // if
    } // while
} // void calibrate


float Tri=0;
float Triangular(){
    float Periodo,A,P;
    Periodo=1; //datos señal triangular
    A=7.36/2; //Amplitud
    //TRIANGULAR 1Hz

     if (samples<(1/(Ts/1e6)))
    {
  P=((4*A)/2)*(Tri);
    }
    else { P=0;}

    if(samples>(1/(Ts/1e6))){ samples=0;}

    Ref=P;
    return Ref;
    }


void GuranteeControl(){
  float delta=0.0;
    if(Ref_VelWR>Rads_Max/2){
    delta  = Rads_Max - Ref_VelWR;
    Velocidad = Velocidad + delta;
    Ref_VelWL=Velocidad - U_Line/2; // Accion sobre motor Izquierdo
    Ref_VelWR=Velocidad + U_Line/2; // Accion sobre motor Derecho
  }
  if(Ref_VelWL>Rads_Max/2){
      delta  = Rads_Max - Ref_VelWL;
      Velocidad = Velocidad + delta;
    Ref_VelWL=Velocidad - U_Line/2; // Accion sobre motor Izquierdo
    Ref_VelWR=Velocidad + U_Line/2; // Accion sobre motor Derecho


  }

}




int main(){
    T.start(); // Start the sistem of measure time.
    BLE.baud(115200); // comunication Rate whit the Bluetooth
    pc.baud(115200); // comunication Rate whit the pc
    blue.baud(115200);// comunication Rate whit the Slave microcontroller
       Turbina.Enable(1000,20000);
    BLE.printf("Hello World LineFollower\n"); // ON Sistem in Bluetooth
    pc.printf("Hello World LineFollower\n"); // ON Sistem in pc
    MI.period_us(1000);      //Frecuency of pwm signal
    MD.period_us(1000);      //Frecuency of pwm signal
    MI1.period_us(1000);     //Frecuency of pwm signal
    MD1.period_us(1000);     //Frecuency of pwm signal
    wait_ms(2000);
    Button_1.mode(PullUp);
    Button_2.mode(PullUp);
    IR.mode(PullUp);

    Kp_Line=20;
    Kd_Line=0.12;




    Kp_L=0.03555068085,Kd_L=0.0002,Ki_L=0.06;
    Kp_R=0.03555068085,Kd_R=0.0002,Ki_R=0.06;
    V_MAX=220.0; /// 250 150 200
    while(1) {
        if(Button_1 == 0 || IR==0){
            Turbina.SetPosition(1600);
            wait_ms(3000);
            while(1){
                samples++;
                start = T.read_us();
                newPosition_L = wheel_L.getPulses();
                if (newPosition_L != oldPosition_L){
                    toc_L=T.read_us();
                    difTime_L=(toc_L-tic_L)/1e6;
                    tic_L=T.read_us();
                    dW_L=(newPosition_L-oldPosition_L)*0.5236;
                    w_L=((dW_L/(difTime_L))/2)/2.7;

                // FILTERING
                W_L[0]=W_L[1];
                W_L[1]=W_L[2];
                W_L[2]=w_L;
                w_L=filt_coef_B[0]*W_L[1]+filt_coef_B[1]*W_L[0]-filt_coef_A[0]*W_L_filtered[1];
                W_L_filtered[0]=W_L_filtered[1];
                W_L_filtered[1]=W_L_filtered[2];
                W_L_filtered[2]=w_L;

                }else{
                    w_L=0;}//
                oldPosition_L = newPosition_L;
                newPosition_R = wheel_R.getPulses();
                if (newPosition_R != oldPosition_R){
                    toc_R=T.read_us();
                    difTime_R=(toc_R-tic_R)/1e6;
                    tic_R=T.read_us();
                    dW_R=(newPosition_R-oldPosition_R)*0.5236;
                    w_R=((dW_R/(difTime_R))/2)/2.7;

               // FILTERING
               W_R[0]=W_R[1];
               W_R[1]=W_R[2];
               W_R[2]=w_R;
               w_R=filt_coef_B[0]*W_R[1]+filt_coef_B[1]*W_R[0]-filt_coef_A[0]*W_R_filtered[1];
               W_R_filtered[0]=W_R_filtered[1];
               W_R_filtered[1]=W_R_filtered[2];
               W_R_filtered[2]=w_R;

               }else{
                    w_R=0;}//
                oldPosition_R = newPosition_R;

                //*****Fin encoders***//
               /*
               if (samples < (0.5/(Ts/1e6))){
                  Ref=7.63;
                  }


               if (samples > (0.5/(Ts/1e6)) && samples < (1/(Ts/1e6))){
                  Ref=0.0;
                  }

               if (samples > (1/(Ts/1e6))){
                  samples=0;
                  }
    */
                        //Ref=Triangular();
                Ref=0;
                         pc.printf("%f\n",Ref);     // Lec action
               Slave_Data=(Linea());
               Clase=Slave_Data/10000;
               line_position = Slave_Data%10000;
               AAA=((line_position-3500)/1e5)/0.17;
               AAA=atan(AAA)*57.29;
               LineError=Ref-AAA;
               U_Line= ( Kp_Line*LineError) + (Kd_Line*(LineError-LineError2)/(Ts/1e6));
               ////
               /*  if (U_Line>=2){U_Line=2;} // Limitacion accion de control
               if (U_Line<=-2) {U_Line=-2;}
               */
               // U_Line=(U_Line/2)*Rads_Max;
               ////
               LineError2=LineError;
               if (U_Line>=Rads_Max){U_Line=Rads_Max;} // Limitacion accion de control
               if (U_Line<=Rads_Min) {U_Line=Rads_Min;}
              Tri=(samples*Ts/1e6);


 /*
   if (samples > (1/(Ts/1e6)) && samples < (1.5/(Ts/1e6))){
    Ref_VelWL=Ref_VelWR=200;
    }
    if (samples > (1.5/(Ts/1e6)) && samples < (2/(Ts/1e6))){
      Ref_VelWL=Ref_VelWR=100;
    }
    if (samples > (2/(Ts/1e6)) && samples < (2.5/(Ts/1e6))){
       Ref_VelWL=Ref_VelWR=-200;
    }
    if (samples > (2.5/(Ts/1e6)) && samples < (3/(Ts/1e6))){
       Ref_VelWL=Ref_VelWR=120;
       samples=0;
     }

    */




               Velocidad=V_MAX;
               Ref_VelWL=Velocidad-U_Line/2;
               Ref_VelWR=Velocidad+U_Line/2;



               //GuranteeControl();



               if (Ref_VelWL>=Rads_Max){Ref_VelWL=Rads_Max;} // Limitacion accion de control
               if (Ref_VelWL<=Rads_Min) {Ref_VelWL=Rads_Min;}
               if (Ref_VelWR>=Rads_Max){Ref_VelWR=Rads_Max;} // Limitacion accion de control
               if (Ref_VelWR<=Rads_Min) {Ref_VelWR=Rads_Min;}






               Error_WL=Ref_VelWL-w_L;
               U_iL=Ki_L*Error_WL*(Ts/1e6) + U_ikL;
               U_wl=(Kp_L*Error_WL)+((Kd_L)*(Error_WL-Error_WL2)/(Ts/1e6)) + U_iL;
               Error_WL2=Error_WL;
               if(U_iL>1.0){U_iL=1.0;}
               if(U_iL<-1.0){U_iL=-1.0;}
               U_ikL=U_iL;


               Error_WR=Ref_VelWR-w_R;
               U_iR=Ki_R*Error_WR*(Ts/1e6) + U_ikR;
               U_wr=(Kp_R*Error_WR)+((Kd_R)*(Error_WR-Error_WR2)/(Ts/1e6)) + U_iR;
               Error_WR2=Error_WR;
               if(U_iR>1.0){U_iR=1.0;}
               if(U_iR<-1.0){U_iR=-1.0;}
               U_ikR=U_iR;

                   if(U_wr>1.0){U_wr=1;}
                   if(U_wr<-1.0){U_wr=-1;}
                   if(U_wl>1.0){U_wl=1;}
                   if(U_wl<-1.0){U_wl=-1;}

                if(U_Line>0){ LED_Der  = 1,LED_Iz  = 0;
                }else if(U_Line<0){ LED_Der  = 0,LED_Iz  = 1;
                }else{ LED_Der  = 1,LED_Iz  = 1;
                }

                UR = U_wr;
                UL = U_wl;



                     if(IR==0 ){UL=UR=0.0;
     samples=0;
     BLE.printf("---------------------");
                Turbina.SetPosition(1000);
                if(UR>=0){ MD1=0,MD.write(abs(UR));
                }else{ MD=0,MD1.write(abs(UR));}
                if(UL<=0){ MI=0,MI1.write(abs(UL));
                }else{  MI1=0,MI.write(abs(UL)); }
                wait_ms(500);
                break;}

                //*****Limitacion Motores***//
                if(UR>1){UR=1;}
                if(UR<-1){UR=-1;}
                if(UL>1){UL=1;}
                if(UL<-1){UL=-1;}
                if(UR>=0){ MD1=0,MD.write(abs(UR));
                }else{ MD=0,MD1.write(abs(UR));}
                if(UL<=0){ MI=0,MI1.write(abs(UL));
                }else{  MI1=0,MI.write(abs(UL)); }





                BLE.printf("%f", Ref);   //Left
                BLE.printf("\t");
                BLE.printf("%f",U_Line);    // Right
                BLE.printf("\t");
                BLE.printf("%f", Ref_VelWL);   //Left
                BLE.printf("\t");
                BLE.printf("%f",Ref_VelWR);    // Right
                BLE.printf("\t");
                BLE.printf("%f",w_L);     // Lec action
                BLE.printf("\t");
                BLE.printf("%f",w_R);     // Right action
                BLE.printf("\t");
                BLE.printf("%f\n",AAA);     // Lec action
                      //** Garantizar perido de Muestreo**//
                        finish = T.read_us() - start;
                        if (finish <Ts){ myled=0;}else{myled=1;}
                        while(finish<=Ts) { finish = T.read_us() - start;}
                    }

    }else if(Button_2 == 0){
      Calibration();
    }
 }
}
