 
/******************************************************************************* 
    * Advanced Line Follower
    * SIRUI
    * Harold F MURCIA, Dennis Santa

    This file content the control of a Line Follower equiped with an engine and 
    a master/slave architecture control. The system is based on a MBED micro, 
    8QTRA pololu line sensors, IMU and encoders in a diferential topology.
    
********************************************************************************/

#include "math.h"
#include "QEI.h"
#include "mbed.h"
#include "Servo.h"
#include "pid.h"
#include <stdio.h>


DigitalOut myled(LED1);         // LED del ARM
DigitalOut LED_LEFT(p26);         // LED1
DigitalOut LED_RIGHT(p18);        // LED2
DigitalIn  Button_1(p7);        // Button_1
DigitalIn  Button_2(p8);        // Button_2
DigitalIn  IR(p9);        // RCIR


float P,E,Periodo,A;

QEI wheel_R(p16, p15, NC, 1);   // Encoder
QEI wheel_L(p5, p6, NC, 1);     // Encoder


Serial pc(USBTX, USBRX);        // USB connection
Serial BLE(p13,p14);            // tx, rx Bluethoot serial
Serial blue(p28,p27);           // UART0_TX, UART0_RX / slave micro-controller
Servo      Turbina(p25);
PwmOut MI1(p24);                // Lef Motor 1
PwmOut MI(p23);                 // Lef Motor 2
PwmOut MD1(p22);                // Rigth Motor 1
PwmOut MD(p21);                 // Rigth Motor 2

float Referencia=350.00;        // Line Reference <==> Angle=0ยบ
float Position_Line = 0.0;

// Filter
const float filt_coef_B[2]={0.112160244475193, 0.112160244475193};
const float filt_coef_A[1]={ -0.775679511049613};
float W_L[3]={0,0,0};
float W_L_filtered[3]={0,0,0};
float W_R[3]={0,0,0};
float W_R_filtered[3]={0,0,0};

Timer T;

float start,finish;
const float Ts = 8000.0;
int lastPosition = 0.0;

//------------ Encoder Variables ---------------//
float difTime_L,difTime_R;
float dW_L,w_L,dW_R,w_R;
long oldPosition_L  =0.0;
long oldPosition_R  =0.0;
int tic_L,toc_L, tic_R,toc_R;

//------------- Variables ---------------------//

// Controller  
float K,z1;
float X1,X2;
float Ref, U, Ud, Ui;
long newPosition_R, newPosition_L;
//const float Kp=0.1098, Ki=0.000, Kd=0.0043;// La hace con Garantia
const float Kp=0.119498, Ki=0.000, Kd=0.0043;
//Kp=10.9; Kd=0.459;
float AAA=0.0;
// Line IR-array
int Slave_Data;
int line_position, line_state; //  1 = White,2 = Black and 3 == line 
//Gurantee control
float U_Base=0, U_MAX = 1.0, U_MIN=-1.0, delta = 0;
float R=1.01/100;
float speed=0;
int Tur=1000;
 
float UR,UL;
int samples=0;

float Car=0;
int muestras=0;
int muestras2=0;
float Tri=0;
int Lecturas[10]; //Variables "filtro"
int i = 0;float Total = 0;float Promedio = 0;
int h = 0;
#define size_buffer 16
char buffer[size_buffer];
uint16_t count_data;

// ***************************************************************************//
//-------------                   Functions             ----------------------//
// ***************************************************************************//

int Linea(){
    for(int i=0;i<size_buffer;i++){
        buffer[i] = NULL;
    }
    count_data = 0;
    blue.putc(1);
    wait_us(100);
    int i;
    
    if(blue.readable()>0){
    //BLE.printf(" sias\n ");
    char buffer [size_buffer];
    //wait_us(100);
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
        LED_RIGHT  = 1,LED_LEFT  = 0;
        wait_ms(350);
        LED_RIGHT  = 0,LED_LEFT  = 1;
        wait_ms(300);
        LED_RIGHT  = 1,LED_LEFT  = 0;
        wait_ms(250);
        LED_RIGHT  = 0,LED_LEFT  = 1;
        wait_ms(200);
        LED_RIGHT  = 1,LED_LEFT  = 1;
        if(h==3){
            pc.printf(" Calibrado");
            LED_LEFT  = 0,LED_RIGHT  = 1;
            wait_ms(350);
            LED_RIGHT  = 0,LED_LEFT  = 1;
            wait_ms(350); 
            LED_LEFT  = 0,LED_RIGHT  = 1;
            wait_ms(350);
            LED_RIGHT  = 0,LED_LEFT  = 1;
            wait_ms(350);
            LED_RIGHT  = 0,LED_LEFT  = 0;
            wait_ms(550);
            LED_RIGHT  = 1,LED_LEFT  = 1;
            wait_ms(750);
            LED_RIGHT  = 0,LED_LEFT  = 0;
            wait_ms(750);
            LED_RIGHT  = 1,LED_LEFT  = 1;
            wait_ms(750);
            LED_RIGHT  = 0,LED_LEFT  = 0;
            wait_ms(750);
            break;
        } // if   
    } // while                   
} // void calibrate



float PBRS(){
      
      if (samples < (0.5/(Ts/1e6))){
         Ref=7.63;
     }
     if (samples > (0.5/(Ts/1e6)) && samples < (1/(Ts/1e6))){
         Ref=0.0;
     }
     
     /*
     if (samples < (0.3/(Ts/1e6))){
         Ref=-7.63;
     }
     if (samples > (0.3/(Ts/1e6)) && samples < (.5/(Ts/1e6))){
         Ref=7.63;
     }
     
     
     if (samples > (0.5/(Ts/1e6)) && samples < (.6/(Ts/1e6))){
         Ref=-7.63;
     }
     
     if (samples > (0.6/(Ts/1e6)) && samples < (0.9/(Ts/1e6))){
         Ref=7.63;
     }
     
     if (samples > (0.9/(Ts/1e6)) && samples < (1.2/(Ts/1e6))){
         Ref=-7.63;
     }
     
     if (samples > (1.2/(Ts/1e6)) && samples < (1.46/(Ts/1e6))){
         Ref=7.63;
     }
     
     if (samples > (1.46/(Ts/1e6)) && samples < (1.57/(Ts/1e6))){
         Ref=-7.63;
     }
     
     
     if (samples > (1.57/(Ts/1e6)) && samples < (1.7/(Ts/1e6))){
         Ref=7.63;
     }
      
      if (samples > (1.7/(Ts/1e6)) && samples < (1.9/(Ts/1e6))){
         Ref=-7.63;
     }
     
      if (samples > (1.9/(Ts/1e6)) && samples < (2.1/(Ts/1e6))){
         Ref=7.63;
     }
     
      if (samples > (2.1/(Ts/1e6)) && samples < (2.25/(Ts/1e6))){
         Ref=-7.63;
     }
     
     if (samples > (2.25/(Ts/1e6)) && samples < (2.65/(Ts/1e6))){
         Ref=7.63;
     }
     
     
     if (samples > (2.65/(Ts/1e6)) && samples < (2.85/(Ts/1e6))){
         Ref=-7.63;
     }
     
     */
     
     if (samples > (1/(Ts/1e6)) ){
         samples=0;
     }
     return Ref;

}


float SENO(){
     float F1=0.04;
     float F2=0.02;
     float pi=3.14159265358979323846;
     //if (samples < (2.5/(Ts/1e6))){
     //Ref=3500.00 + 2000*sin(2*pi*F1*samples);
     //}else{
         Ref=3500.00 + 2000*sin(2*pi*F1*samples);
       //  }
     
     
     return Ref;
    }



float Triangular(){
    
    Periodo=0.5; //datos señal triangular
    A=7.36; //Amplitud
    //TRIANGULAR 1Hz
       
    
    if (muestras<(0.5/(Ts/1e6)))
    {
    P=((4*A)/1)*(Tri)-A;
    }
    if (muestras>(0.5/(Ts/1e6)) && muestras < (1/(Ts/1e6)))
    {
    P=((-4*A)/1)*(Tri)+3*A;
    }
    
    
    if (samples > (1/(Ts/1e6)) && samples < (1.5/(Ts/1e6))){
       P=7.36;
    }
    if (samples > (1.5/(Ts/1e6)) && samples < (2/(Ts/1e6))){
       P=-7.36;
    }
    if (samples > (2/(Ts/1e6)) && samples < (2.5/(Ts/1e6))){
       P=7.36;
    }
    if (samples > (2.5/(Ts/1e6)) && samples < (3/(Ts/1e6))){
       P=-7.36;
     }
     
    
    if (muestras > (3/(Ts/1e6)) && muestras < (3.25/(Ts/1e6)))
    {
    P=((4*A)/0.5)*(Car)-A;
    }
    if (muestras > (3.25/(Ts/1e6)) && muestras < (3.5/(Ts/1e6)))
    {
    P=((-4*A)/0.5)*(Car)+3*A;
    }
    
  
    Ref=P;  
    return Ref;
    }


void GuranteeControl(){
    
        
  if(UR>U_MAX){
    delta  = U_MAX - UR;
    U_Base = U_Base + delta;
    UL=U_Base - U/2; // Accion sobre motor Izquierdo
    UR=U_Base + U/2; // Accion sobre motor Derecho
  }
  if(UL>U_MAX){
      delta  = U_MAX - UL;
      U_Base = U_Base + delta;
    UL=U_Base - U/2; // Accion sobre motor Izquierdo
    UR=U_Base + U/2; // Accion sobre motor Derecho
  
    }
    
}


// ***************************************************************************//
//------------                  MAIN                   -----------------------//
// ***************************************************************************//
int main(){
    T.start();                      // Starts time capture.
    myled=1;
    BLE.baud(115200);               // Data trasnfer setup Bluethooth.
    pc.baud(115200);                // Data transfer setup PC - USB
    blue.baud(115200);
    BLE.printf("Hello World LineFollower\n");
    pc.printf("Hello World LineFollower\n");
    MI.period_us(1000);      
    MD.period_us(1000);      
    MI1.period_us(1000);      
    MD1.period_us(1000);      
   Turbina.Enable(1000,20000);
    Button_1.mode(PullUp);
    Button_2.mode(PullUp);
    IR.mode(PullUp);

    Ref=0.0;
    // Setup Controller
    PID pid_line = PID(Ts/1e6, U_MAX, U_MIN, Kp, Kd, Ki); 
    
    wait_ms(1000);
    myled=0;
    while(1) {
        IR.mode(PullUp);
        if(Button_1 == 0 || IR==0){
             pc.printf("---------------------");
            Turbina.SetPosition(1300);
             wait_ms(1000);
            while(1){
                start = T.read_us();
                samples++;
                 //**** Encoders****//          
                newPosition_L = wheel_L.getPulses();
                if (newPosition_L != oldPosition_L){
                    toc_L=T.read_us();
                    difTime_L=(toc_L-tic_L)/1e6;
                    tic_L=T.read_us();
                    dW_L=(newPosition_L-oldPosition_L)*0.5236;
                    w_L=((dW_L/(difTime_L))/2)/2.7;
                    /*
                    // FILTERING
                    W_L[0]=W_L[1];
                    W_L[1]=W_L[2];
                    W_L[2]=w_L;
                    w_L=filt_coef_B[0]*W_L[1]+filt_coef_B[1]*W_L[0]-filt_coef_A[0]*W_L_filtered[1];
                    W_L_filtered[0]=W_L_filtered[1];
                    W_L_filtered[1]=W_L_filtered[2];
                    W_L_filtered[2]=w_L;
                     */
                }else{
                    w_L=0;
                }
                oldPosition_L = newPosition_L;    
                newPosition_R = wheel_R.getPulses();
                if (newPosition_R != oldPosition_R){
                    toc_R=T.read_us();
                    difTime_R=(toc_R-tic_R)/1e6;
                    tic_R=T.read_us();
                    dW_R=(newPosition_R-oldPosition_R)*0.5236;
                    w_R=((dW_R/(difTime_R))/2)/2.7;
                    /*
                    // FILTERING
                    W_R[0]=W_R[1];
                    W_R[1]=W_R[2];
                    W_R[2]=w_R;
                    w_R=filt_coef_B[0]*W_R[1]+filt_coef_B[1]*W_R[0]-filt_coef_A[0]*W_R_filtered[1];
                    W_R_filtered[0]=W_R_filtered[1];
                    W_R_filtered[1]=W_R_filtered[2];
                    W_R_filtered[2]=w_R;
                    */
                }else{
                    w_R=0;
                }
                oldPosition_R = newPosition_R; 
                //*****Fin encoders***//
    
    R=1.01/100;
    speed = R*(w_R+w_L)/2;
    Tur = 1400 + 350*(speed*speed);
    if (Tur>1700){
     Tur = 1700;
    }
    
                             
               
               
                Slave_Data    = (Linea());
                line_state    = Slave_Data/10000;
                line_position = Slave_Data%10000;
                AAA=((line_position-3500)/1e5)/0.17;
                AAA=atan(AAA)*57.29;
                //  Ref=Triangular();
                Ref =PBRS();
                //Ref=0.0;
                U = pid_line.calculate(Ref, (AAA));
                
                if (U>2.0){U=2.0;}
                if (U<-2.0){U=-2.0;}
                if(U>0){ LED_RIGHT  = 1; LED_LEFT  = 0;
                }else if(U<0){ LED_RIGHT  = 0; LED_LEFT  = 1;
                }else{ LED_RIGHT  = 1; LED_LEFT  = 1;
                }
                
                U_Base=0.4; //1  0.5  0.7 0.3 
                UR = U_Base + U/2;  
                UL = U_Base - U/2;   
                //GuranteeControl();
                
                muestras++;  
                Tri=(muestras*Ts/1e6);
                
                
                if( samples>(3/(Ts/1e6))&& samples<(3.5/(Ts/1e6))){
                    muestras2++;  
                    }
                Car=(muestras2*Ts/1e6);
             /*if(samples>(3.5/(Ts/1e6))){ muestras=0,Tri=0.0,i=0,Car=0,muestras2=0,samples=0;
             BLE.printf("---------------------");
                Turbina.SetPosition(1000);
                if(UR>=0){ MD1=0,MD.write(abs(UR));
                }else{ MD=0,MD1.write(abs(UR));}
                if(UL<=0){ MI=0,MI1.write(abs(UL));
                }else{  MI1=0,MI.write(abs(UL)); }
                wait_ms(500);
                break;
             
             } */
           
       
               
                
     pc.printf("%f \n", Ref);   //Clase  1, blanco, 2 negro, 3 linea
     
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
               
                if(UR>1){UR=1;}
                if(UR<-1){UR=-1;}
                if(UL>1){UL=1;}
                if(UL<-1){UL=-1;}
    
                if(UR>=0){ MD1=0,MD.write(abs(UR));
                }else{ MD=0,MD1.write(abs(UR));}
                if(UL<=0){ MI=0,MI1.write(abs(UL));
                }else{  MI1=0,MI.write(abs(UL)); }
                
              
            
                BLE.printf("%f", Ref);   //Clase  1, blanco, 2 negro, 3 linea
                BLE.printf("\t");
                BLE.printf("%f", AAA);   //Reference
                BLE.printf("\t");
                BLE.printf("%f", w_L);   //Left
                BLE.printf("\t");
                BLE.printf("%f",w_R);    // Right
                BLE.printf("\t");
                BLE.printf("%f",UL);     // Lec action
                BLE.printf("\t");
                BLE.printf("%f",UR);     // Right action
                BLE.printf("\t");
                BLE.printf("%f\n",U);     // Lec action
    
                //** Garantizar perido de Muestreo**//
                finish = T.read_us() - start;
                if (finish <Ts){ myled=0;}else{myled=1;}
                while(finish<=Ts) { finish = T.read_us() - start;}
            }
       
    }else if(Button_2 == 0 ){
    
      
      
      Calibration();  
        BLE.printf(" Turbina 1");
           
    }
 }
}
