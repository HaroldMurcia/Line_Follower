   #include <QTRSensors.h>

#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

long Cod;
int sum;
int Clase=3; 
float aux;
float mean,aux2,Desv;

uint16_t position_line; 

QTRSensorsAnalog qtra((unsigned char[]) {5,4,3,2,1,0,7,6}, 
 NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

int estado=0;
void setup()
{
  Serial.begin(115200);
//Serial.print("Hola");
delay(500);
//Serial.flush();
while(1) {
    if(Serial.available()>0){
         estado = (Serial.read());      
         if (estado>0){
    for (int i = 0; i < 300; i++)  // make the calibration take about 10 seconds
  {

    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)

//Serial.println("Hola");
}
break;

    
  }


 
 }}
 
 while(1){

   
    if(Serial.available()>0){
         estado = (Serial.read());      
         if (estado>0){
           Serial.println(3);
   estado=0;    
           break;
         }}
   
   

   }
 
 
   
   
  
}


void loop()
{
   if(Serial.available()>0){
   estado = (Serial.read());      
  if (estado>0){

           qtra.read(sensorValues);
//int SetP=700;
//if(sensorValues[0]>SetP && sensorValues[1]>SetP && sensorValues[6]>SetP && sensorValues[7]>SetP){
  
//position_line = qtra.readLine(sensorValues, QTR_EMITTERS_ON, 1); // linea Blanca
//}else{
 position_line = qtra.readLine(sensorValues); 
//}

mean=(sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7])/8;
for(int i=0;i<7;i++){
	aux=pow((sensorValues[i]-mean),2);
	aux2=(aux+aux2);
	}
Desv=((aux2)*0.1428);


if (mean<50 && Desv<400){
  Clase=1;
} else if(mean>800 && Desv>50 && Desv<100){
  Clase=2;
} else if(mean>150 && Desv>100000){
  Clase=3;
}          
Cod=10e3*(long)Clase + (long)position_line; // Concatenacion de posicion y clase
/*Serial.print(mean);
Serial.print('\t');
Serial.print(Desv);
Serial.print('\t');
*/
Serial.println(Cod);

aux2=0;
aux=0;

}

  }
  
}

