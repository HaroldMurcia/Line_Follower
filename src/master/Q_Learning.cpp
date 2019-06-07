
/******************************************************************************* 
 
********************************************************************************/

#include "math.h"
#include "QEI.h"
#include "mbed.h"
#include "Servo.h"
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
int variable;
float Referencia=350.00;        // Line Reference <==> Angle=0ยบ
float Position_Line = 0.0;
int POS=0;
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
float Dev;
float Y_1,Y;
float Ref, U, Ud, Ui;
long newPosition_R, newPosition_L;
// Q Learning Variables

int Tarjet=2;
int action = 0;
int num_random = 0;
int num_max = 0;
int n=0;
int SEL;
int index = 0;
float alfa=0.4;
float Gmm=0.4;
int St,St_1=0.0;
int Est=0;
int Dt,Dt_1=0.0;
int R [15][5]=
{

// Acciones  
    //GirarIzq,GirarIzqSuave,Avanzar,GirarDerSuave,GirarDer\\

                                //Estados

    {0,  0,   0,  4, 0}, //Sii1
    {0,  0,   0,  0, 4},//Sii2
    {0,  0,   0,  0, 4},//Sii3
    {0,  0,   0,  4, 0},//Si1
    {0,  0,   0,  4, 0},//Si2
    {0,  0,   0,  4, 0},//Si3
    {0,  0,   15,  0, 0},//SC1
    {0,  0,   40,  0, 0},//SC2
    {0,  0,   15,  0, 0},//SC3
    {4,  0,   0,  0, 0},//Sd1
    {0,  4,   0,  0, 0},//Sd2
    {0,  4,   0,  0, 0},//Sd3
    {0,  4,   0,  0, 0},//Sdd1
    {4,  0,   0,  0, 0},//Sdd2
    {4,  0,   0,  0, 0},//Sdd3


};

int Q[15][5]=
{

    {0,  0,   0,  0, 0}, //Sii1
    {0,  0,   0,  0, 0},//Sii2
    {0,  0,   0,  0, 0},//Sii3
    {0,  0,   0,  0, 0},//Si1
    {0,  0,   0,  0, 0},//Si2
    {0,  0,   0,  0, 0},//Si3
    {0,  0,   0,  0, 0},//SC1
    {0,  0,   0,  0, 0},//SC2
    {0,  0,   0,  0, 0},//SC3
    {0,  0,   0,  0, 0},//Sd1
    {0,  0,   0,  0, 0},//Sd2
    {0,  0,   0,  0, 0},//Sd3
    {0,  0,   0,  0, 0},//Sdd1
    {0,  0,   0,  0, 0},//Sdd2
    {0,  0,   0,  0, 0},//Sdd3
};

float Q_1;
float Q_2;






int F=0,C=0;

int MAX( int A,int B, int C, int D, int E){
    int NUM=0;
    if(A > B && A > C  && A>D && A>E){
    NUM=A;
    }else{
        if(B>A && B>C && B>D && B>E){
                NUM=B;
                }else{
                if(C>A && C>B && C>D && C>E){
                NUM=C;
                }else{
                if(D>A && D>B && D>C && D>E){
        
                NUM=D;
                }else{
                NUM=E;
        }}
        }
    }
   return NUM;
}



int MAX_P( int A,int B, int C, int D, int E){
    
    if(A > B && A > C  && A>D && A>E){
    POS=0;
    }else{
        if(B>A && B>C && B>D && B>E){
                POS=1;
        }else{
                if(C>A && C>B && C>D && C>E){
                POS=2;
        }else{
                if(D>A && D>B && D>C && D>E){
                POS=3;
        }else{
                POS=4;
        }}
        }
    }
   return POS;
}




//--------\\

// Line IR-array
int Slave_Data;
int line_position, line_state; //  1 = White,2 = Black and 3 == line 

//Gurantee control
float U_Base=0, U_MAX = 1, U_MIN=-1, delta = 0;

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
    
void Motors(float Ul,float Ur)          {
          
          UL=Ul;
          UR=Ur;
          if(UR>1){UR=1;}
                if(UR<-1){UR=-1;}
                if(UL>1){UL=1;}
                if(UL<-1){UL=-1;}
    
                if(UR>=0){ MD1=0,MD.write(abs(UR));
                }else{ MD=0,MD1.write(abs(UR));}
                if(UL<=0){ MI=0,MI1.write(abs(UL));
                }else{  MI1=0,MI.write(abs(UL)); }
          }
          
                   
int Estado(){
     
                          
        Slave_Data    = (Linea());
        line_state    = Slave_Data/10000;
        line_position = Slave_Data%10000;
        
         int VAR=0;
         int AUX1=0;
            AUX1=Est=0;
         
        Y=line_position;
        Dev=Y_1-Y;
        Y_1=Y;
        
        
       if (line_position >0 && line_position<=1500) {AUX1=0;}
       if (line_position >1500 && line_position<=3000){AUX1=1;}
       if (line_position >3000 && line_position<=4000) {AUX1=2;}
       if (line_position >4000 && line_position<=5500) {AUX1=3;}
       if (line_position >5500 && line_position<=7000) {AUX1=4;}
           
       if (Dev<0){Est=1;}
       if (Dev==0){Est=2;}
       if (Dev>0){Est=3;}
       
     
       
       /*
       pc.printf("%i\t",Est);
       pc.printf("|-------- %f | \n",Y); 
       */
       if (AUX1==0 && Est==1){VAR=0;}
       if (AUX1==0 && Est==2){VAR=1;}
       if (AUX1==0 && Est==3){VAR=2;}
       if (AUX1==1 && Est==1){VAR=3;}
       if (AUX1==1 && Est==2){VAR=4;}
       if (AUX1==1 && Est==3){VAR=5;}
       if (AUX1==2 && Est==1){VAR=6;}
       if (AUX1==2 && Est==2){VAR=7;}
       if (AUX1==2 && Est==3){VAR=8;}
       if (AUX1==3 && Est==1){VAR=9;}
       if (AUX1==3 && Est==2){VAR=10;}
       if (AUX1==3 && Est==3){VAR=11;}
       if (AUX1==4 && Est==1){VAR=12;}
       if (AUX1==4 && Est==2){VAR=13;}
       if (AUX1==4 && Est==3){VAR=14;}
      
     
       

        return VAR;
          }
          
          
          
    
          
          
          
          
      float V=1.0 ;      
void AC(int n){
    switch(n){
      
        case 0:
            Motors(-0.66*V,0.66*V);
     //wait_ms(80);
            break;
        case 1:
            Motors(-0.35*V,0.46*V);
     //wait_ms(80);
            break;
        case 2:
            Motors(0.66*V,0.66*V);
     //wait_ms(80);
            break;
        case 3:
            Motors(0.46*V,-0.35*V);
     //wait_ms(80);
            break;
        case 4:
            Motors(0.66*V,-0.66*V);
     //wait_ms(80);
            break;

            }
            }


void ACe(int n){
    switch(n){
      
        case 0:
            Motors(-0.66*V,0.66*V);
wait_ms(80);
            break;
        case 1:
            Motors(-0.3*V,0.56*V);
wait_ms(80);
            break;
        case 2:
            Motors(0.0*V,0.0*V);
wait_ms(80);
            break;
        case 3:
            Motors(0.56*V,-0.3*V);
wait_ms(80);
            break;
        case 4:
            Motors(0.66*V,-0.66*V);
     //wait_ms(80);
            break;

            }
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

    
    myled=0;
    while(1) {
        if(Button_1 == 0){
           St=Estado();
                 
           pc.printf("Comenzando Entrenamiento...\n");
           wait_ms(500);
           pc.printf("Ready ! !\n");
           
           while (1){
                samples++;
                St=Estado();     
                //   variable = limite_inferior + rand() % (limite_superior +1 - limite_inferior) ;                    
                 Dt = 0 + rand() % (4 +1 - 0) ;
                 ACe(Dt);
                 St_1=Estado();
                 
                 Q[St][Dt]=R[St][Dt] +Gmm*(MAX_P(Q[St_1][0],Q[St_1][1],Q[St_1][2],Q[St_1][3],Q[St_1][4]));
                 Q_2=Q_1;
                 Q_1=(Q[0][0] + Q[0][1] + Q[0][2] + Q[0][3] + Q[0][4]+
                        Q[1][0] + Q[1][1] + Q[1][2] + Q[1][3] + Q[1][4]+
                        Q[2][0] + Q[2][1] + Q[2][2] + Q[2][3] + Q[2][4]+
                        Q[3][0] + Q[3][1] + Q[3][2] + Q[3][3] + Q[3][4]+
                        Q[4][0] + Q[4][1] + Q[4][2] + Q[4][3] + Q[4][4]+
                        Q[5][0] + Q[5][1] + Q[5][2] + Q[5][3] + Q[5][4]+
                        Q[6][0] + Q[6][1] + Q[6][2] + Q[6][3] + Q[6][4]+
                        Q[7][0] + Q[7][1] + Q[7][2] + Q[7][3] + Q[7][4]+
                        Q[8][0] + Q[8][1] + Q[8][2] + Q[8][3] + Q[8][4]+
                        Q[9][0] + Q[9][1] + Q[9][2] + Q[9][3] + Q[9][4]+
                        Q[10][0] + Q[10][1] + Q[10][2] + Q[10][3] + Q[10][4]+
                        Q[11][0] + Q[11][1] + Q[11][2] + Q[11][3] + Q[11][4]+
                        Q[12][0] + Q[12][1] + Q[12][2] + Q[12][3] + Q[12][4]+
                        Q[13][0] + Q[13][1] + Q[13][2] + Q[13][3] + Q[13][4]+
                        Q[14][0] + Q[14][1] + Q[14][2] + Q[14][3] + Q[14][4])/75.0;
        
        
        
                 if (samples >500){
                    if (Q_1==Q_2){
                        break;}}
    pc.printf("%f\n",Q_1);      
                    
                    
    /*pc.printf("| Estado =%i | \t",St);                  
    pc.printf("%i\t",St_1);
    pc.printf("| %f | \n",Y);      
  
                
               for (F=0; F<15; F++) {
                    for (C=0; C<5; C++) {
                        pc.printf("| %d |",Q[F][C]);
                    }
                    pc.printf("\n");
                }
                pc.printf("-------------\n");
*/
}

    ACe(2);
    LED_RIGHT  = 1,LED_LEFT  = 1;
    pc.printf("Entrenamiento Terminado...\n");
    
        while (1)
        {
pc.printf("%f\n",Q_1);                 
        Button_1.mode(PullUp);
        Button_2.mode(PullUp);
        if (Button_1==0) break;
        }
            
            
while(1){
        start = T.read_us();
        samples++;
        St=Estado();
                
        //variable = limite_inferior + rand() % (limite_superior +1 - limite_inferior) ;          
        SEL=MAX_P(R[St][0],R[St][1],R[St][2],R[St][3],R[St][4]);
        //SEL=MAX_P(R[St][0],R[St][1],R[St][2],R[St][3],R[St][4]);
        SEL=4-SEL;
    
        if(SEL==0 && St==3) SEL=3;
    
        Dt=SEL;
    
    
        St_1=St;
    
        BLE.printf("%f\t", Y);   //Clase  1, bla
        BLE.printf("%i\t", St);   //Clase  1, bla
        BLE.printf("%i\n", Dt);   //Clase  1, bla
        AC( Dt);
        finish = T.read_us() - start;
        if (finish <Ts){ myled=0;}else{myled=1;}
        while(finish<=Ts) { finish = T.read_us() - start;}
           }
       
    }else if(Button_2 == 0){
    
      Calibration();  
                
        }
    }
    }
