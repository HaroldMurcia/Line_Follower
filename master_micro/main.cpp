
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
const float Kp=0.001091, Ki=0.000, Kd=0.000055;

// Line IR-array
int Slave_Data;
int line_position, line_state; //  1 = White,2 = Black and 3 == line 

//Gurantee control
float U_Base=0, U_MAX = 1, U_MIN=-1, delta = 0;
float R=1.01/100;
float speed=0;
int Tur=1000;
 
float UR,UL;
int samples=0;

int Lecturas[10]; //Variables "filtro"
int i = 0;float Total = 0;float Promedio = 0;
int h = 0;
#define size_buffer 256
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
    int i;
    //wait_us(100);
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
     if (samples > (1/(Ts/1e6)) && samples < (3/(Ts/1e6))){
         Ref=4500.0;
     }
     if (samples > (3/(Ts/1e6)) && samples < (4/(Ts/1e6))){
         Ref=2500.0;
     }
     if (samples > (4/(Ts/1e6)) && samples < (5/(Ts/1e6))){
         Ref=4500.0;
     }
     if (samples > (5/(Ts/1e6)) && samples < (5.5/(Ts/1e6))){
         Ref=2500.0;
     }
     if (samples > (5.5/(Ts/1e6)) && samples < (6.5/(Ts/1e6))){
         Ref=4500.0;
     }
     if (samples > (6.5/(Ts/1e6)) && samples < (6.75/(Ts/1e6))){
         Ref=2500.0;
     }
     if (samples > (6.75/(Ts/1e6)) && samples < (7.0/(Ts/1e6))){
         Ref=4500.0;
     }
     if (samples > (7.0/(Ts/1e6)) && samples < (7.1/(Ts/1e6))){
         Ref=2500.0;
     }
     if (samples > (7.1/(Ts/1e6)) && samples < (8.0/(Ts/1e6))){
         Ref=4500.0;
     }
     if (samples > (8.0/(Ts/1e6)) && samples < (8.1/(Ts/1e6))){
         Ref=2500.0;
     }
     if (samples > (8.1/(Ts/1e6)) && samples < (8.5/(Ts/1e6))){
         Ref=4500.0;
     }
     if (samples > (8.5/(Ts/1e6)) && samples < (10.0/(Ts/1e6))){
         Ref=2500.0;
     }  
     if ( samples > (10.0/(Ts/1e6)) ){
         Ref=3500.0;
     }
     return Ref;

}

void GuranteeControl(){
    if(UR>U_MAX){
        delta  = U_MAX - UR;
        U_Base = U_Base+ delta;
    }
    if(UL>U_MAX){
        delta  = U_MAX - UL;
        U_Base = U_Base+ delta;
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
    MI.period_us(100);      
    MD.period_us(100);      
    MI1.period_us(100);      
    MD1.period_us(100);      
   Turbina.Enable(1000,20000);
    Button_1.mode(PullUp);
    Button_2.mode(PullUp);

    Ref=0.0;

    // Setup Controller
    PID pid_line = PID(Ts/1e6, U_MAX, U_MIN, Kp, Kd, Ki); 
    
    wait_ms(1000);
    myled=0;
    while(1) {
        if(Button_1 == 0){
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
                    
                    // FILTERING
                    W_L[0]=W_L[1];
                    W_L[1]=W_L[2];
                    W_L[2]=w_L;
                    w_L=filt_coef_B[0]*W_L[1]+filt_coef_B[1]*W_L[0]-filt_coef_A[0]*W_L_filtered[1];
                    W_L_filtered[0]=W_L_filtered[1];
                    W_L_filtered[1]=W_L_filtered[2];
                    W_L_filtered[2]=w_L;
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
                    
                    // FILTERING
                    W_R[0]=W_R[1];
                    W_R[1]=W_R[2];
                    W_R[2]=w_R;
                    w_R=filt_coef_B[0]*W_R[1]+filt_coef_B[1]*W_R[0]-filt_coef_A[0]*W_R_filtered[1];
                    W_R_filtered[0]=W_R_filtered[1];
                    W_R_filtered[1]=W_R_filtered[2];
                    W_R_filtered[2]=w_R;
                    
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
    
     
     Turbina.SetPosition(Tur);
    
                // Angle control
                
                
                Slave_Data    = (Linea());
                line_state    = Slave_Data/10000;
                line_position = Slave_Data%10000;
                
                Ref = 3500.0;
                //Ref = PBRS();
                
                U = pid_line.calculate(Ref, (line_position));

                if(U>0){ LED_RIGHT  = 1; LED_LEFT  = 0;
                }else if(U<0){ LED_RIGHT  = 0; LED_LEFT  = 1;
                }else{ LED_RIGHT  = 1; LED_LEFT  = 1;
                }
                U_Base=0.78;
                
                if (line_state > 3)
                    line_state = 0;
                
                if(line_state==3 || line_state==1){
                    //GuranteeControl();
                    UR = U_Base + U/2.0;  
                    UL = U_Base - U/2.0;   
                }else{
                    UR = 0;
                    UL = 0;
                }
           
    
                
                 
                //*****Limitacion Motores***//
                if(UR>1){UR=1;}
                if(UR<-1){UR=-1;}
                if(UL>1){UL=1;}
                if(UL<-1){UL=-1;}
    
                if(UR>=0){ MD1=0,MD.write(abs(UR));
                }else{ MD=0,MD1.write(abs(UR));}
                if(UL<=0){ MI=0,MI1.write(abs(UL));
                }else{  MI1=0,MI.write(abs(UL)); }
                
  
            
              BLE.printf("%i", line_state);   //Clase  1, blanco, 2 negro, 3 linea
                BLE.printf("\t");
                BLE.printf("%i", line_position);   //Reference
                BLE.printf("\t");
                BLE.printf("%f", w_L);   //Left
                BLE.printf("\t");
                BLE.printf("%f",w_R);    // Right
                BLE.printf("\t");
                BLE.printf("%f",UL);     // Lec action
                BLE.printf("\t");
                BLE.printf("%f",UR);     // Right action
                BLE.printf("\t");
                BLE.printf("%i",Tur);     // Right action
                BLE.printf("\t");
                BLE.printf("%f\n",U);     // Lec action
  
        /*
              
              
                pc.printf("%i", line_state);   //Clase  1, blanco, 2 negro, 3 linea
                pc.printf("\t");
                pc.printf("%i", line_position);   //Reference
                pc.printf("\t");
                pc.printf("%f", w_L);   //Left
                pc.printf("\t");
                pc.printf("%f",w_R);    // Right
                pc.printf("\t");
                pc.printf("%f",UL);     // Left action
                pc.printf("\t");
                pc.printf("%f",UR);     // Right action
                pc.printf("\t");
                pc.printf("%i",Tur);     // Right action
                pc.printf("\t");
                pc.printf("%f\n",U);     
*/
                //** Garantizar perido de Muestreo**//
                finish = T.read_us() - start;
                if (finish <Ts){ myled=0;}else{myled=1;}
                while(finish<=Ts) { finish = T.read_us() - start;}
            }
       
    }else if(Button_2 == 0){
    
      
      
      Calibration();  
        BLE.printf(" Turbina 1");
           
    }
 }
}
