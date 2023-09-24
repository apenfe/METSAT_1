/*
  METSAT-1 DATALOGGER

  Toma datos de: sensor DHT-11, BMP-180, DS18B20, MPU y GPS. 
  Posteriormente pasa esos datos a un archivo .txt, el cual 
  se cierra a unas determinadas escrituras y se abre otro nuevo 
  para seguir cuardando los datos, lo repite un total de 16 veces 
  hasta que ya se abre el fichero final en el que se guardarán todos
  los datos hasta que el Arduino quede sin batería.

  Todos los componentes electrónicos estaán alimentados a 5(V) y 
  están todos conectados a una masa común.

  Los sensores MPU y BMP-180 están conectados al bus de comms I2C
  por lo que se conectan respectivamente al SDA y SCL (D20 Y D21).

  El GPS está conectado a 5v, GND y luego rx y tx conectado a 
  TXD2 Y RXD2 (D16 Y D17)

  El sensor DHT-11 está conectado al pin digital 5
  El sensor DS18B20 está conectado al pin digital 6

  El lector de tarjetas MicroSD está conectado así:
  MOSI ---- BUSMOSI D51
  MISO ---- BUSMISO D50
  SCK  ---- BUSSCK D52
  CS   ---- D4

  Creado el 11 de enero de 2020
  por Adrián Peñalver Fernández
  modificado el 31 de enero de 2020
  por Adrián Peñalver Fernández
  modificado el 14 de febrero de 2020
  por Germán Villalba
  modificado el 16 de febrero de 2020
  por Adrián Peñalver

  Este codigo es de dominio público.

  https://www.verkami.com/locale/es/projects/25687-ayudanos-a-llegar-a-la-estratosfera
*/

#include "DHT.h"  // Libreria DHT11
#include <OneWire.h> // Libreria BUSCOM
#include <DallasTemperature.h> // Libreria TEMPERATURA
#include <SFE_BMP180.h> // Libreria BAROMETRO
#include <Wire.h> // Libreria BUSCOM
#include <SPI.h> // Libreria BUSSPI
#include <SD.h> // Libreria SD
#include <TinyGPS.h> // Libreria GPS
#include <EEPROM.h>


#define DEBUG 1   // COMENTAR CUANDO SE VA A LANZAR
#define TIME_FILE 600000   // 600000


#define LED_SD_OPEN 1
#define LED_SD_CLOSE 2
#define DHTTYPE DHT11
#define MPU 0x68 
#define A_R 16384.0 //Definimos los ratios de conversión
#define G_R 131.0 
#define RAD_A_DEG = 57.295779 //Definimos la conversion de radianes a grados 180/PI

const int BUZZ = 40; 
const int LED_1 = 44; 
const int LED_2 = 38;
const int PUL_1 = 28;     
const int PUL_2 = 29;
const int chipSelect = 4;   // SD chipSelect
const int oneWirePin = 2;   // DS18D20 
const int DHTPin     = 5;   // DHT-11
int indice;
int x;
int k; 
int addr_k = 0;
char status;
char filename[] = "metsat00.txt";
char borrafile[] = "metsat00.txt";
int dato; 
double T,P;

int16_t AcX; 
int16_t AcY; 
int16_t AcZ; 
int16_t GyX; 
int16_t GyY; 
int16_t GyZ; 
 
//Declaramos cadena de caracteres para los ángulos
float Acc[2]; 
float Gy[2];
float Angle[2]; 
float h; 
float t;
float flat, flon;
float tamano;

unsigned long age;
unsigned long ldate, ltime, laltitude, lage, lcourse, lspeed;
//unsigned long antTime = millis();

SFE_BMP180 bmp180; // OBJETO BMP180
DHT dht(DHTPin, DHTTYPE); 
OneWire oneWireBus(oneWirePin);
DallasTemperature sensor(&oneWireBus);
File logFile;
TinyGPS gps; // OBJETO GPS
 
void setup() {
  
   pinMode(BUZZ, OUTPUT); // pin 33
   pinMode(LED_1, OUTPUT); // pin 44
   pinMode(LED_2, OUTPUT); // pin 38
   pinMode(PUL_1, OUTPUT); // pin 28
   pinMode(PUL_2,INPUT_PULLUP); // pin 29
   digitalWrite(PUL_1, LOW);
  
    Wire.begin(); 
    Wire.beginTransmission(MPU); 
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true); 
    
    Serial.begin(9600);
 
    sensor.begin(); 
    dht.begin();
    bmp180.begin();

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
      
       #if DEBUG
       Serial.println("CardFail");
        #endif
       // don't do anything more:
       while (1);
     }

    #ifdef DEBUG
      Serial.println("cardInit");
    #endif

    Serial2.begin(4800); 

    if(EEPROM.read(addr_k) > 0){
      k = EEPROM.read(addr_k);
      
       #ifdef DEBUG
       Serial.print("SE HA RESETEADO");
       #endif
       
       
      }
    else{
      
      #ifdef DEBUG
      Serial.print("SE INICIA POR PRIMERA VEZ");
       #endif
       
      k=0;
      }
}   
void readTempHumedad() {

   h = dht.readHumidity();
   t = dht.readTemperature();
   
   sensor.requestTemperatures();
} 

void readBMP(){

  status = bmp180.startTemperature();//Inicio de lectura de temperatura
 
  if (status != 0)
  {   
    delay(status); //Pausa para que finalice la lectura
    status = bmp180.getTemperature(T); //Obtener la temperatura
    if (status != 0)
    {status = bmp180.startPressure(3); //Inicio lectura de presión
      if (status != 0)
      {delay(status);//Pausa para que finalice la lectura        
        status = bmp180.getPressure(P,T); //Obtenemos la presión       
      }      
    }   
  } 
  
}


void readAccGyro(){

    // valores del Acelerometro
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedimos el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   // valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,4,true); //A diferencia del Acelerometro, solo se piden 4 registros
   GyX=Wire.read()<<8|Wire.read();
   GyY=Wire.read()<<8|Wire.read();
}

void printSDcard(){ 

        logFile.print("$"); logFile.print(",");
        logFile.print(millis()/1000); logFile.print(",");
        logFile.print(h); logFile.print(",");
        logFile.print(t); logFile.print(",");
        logFile.print(sensor.getTempCByIndex(0)); logFile.print(",");
        logFile.print(T,2); logFile.print(",");
        logFile.print(P,2); logFile.print(","); 
        logFile.print(AcX); logFile.print(",");
        logFile.print(AcY); logFile.print(",");
        logFile.print(AcZ); logFile.print(",");
        logFile.print(GyX); logFile.print(",");
        logFile.print(GyY); logFile.print(",");
        logFile.print(GyZ); logFile.print(",");
        logFile.print(flat); logFile.print(",");
        logFile.print(flon); logFile.print(",");
        logFile.print(ldate); logFile.print(",");
        logFile.print(ltime); logFile.print(",");
        logFile.print(laltitude); logFile.print(",");
        logFile.print(lage); logFile.print(",");
        logFile.print(lcourse); logFile.print(",");
        logFile.print(lspeed); logFile.print(",");
        logFile.println("#");
 
        logFile.close();
}

void dualLed(int x){
 
   if(x==1){     // verde seguro
    digitalWrite(LED_1, LOW);
       digitalWrite(LED_2,HIGH);
       }
   else if(x==2){ // rojo peligro
       digitalWrite(LED_1, HIGH);
       digitalWrite(LED_2,LOW);
        }
   else{
       digitalWrite(LED_1, LOW);
       digitalWrite(LED_2,LOW);
       }
}

void lecturaSd(){

   if(Serial.available() > 0){
     
     dato = Serial.read();
    }

    if( dato == '1'){
      Serial.println("--------------------------------------------------------------------------------------------------");
       Serial.println("");
           Serial.println("OPCIÓN 1: LECTURA DE DATOS");
           Serial.println("");
           Serial.flush();
           delay(2000);
             tamano=0;
             indice=0;
            while(indice<=15){
           sprintf(filename, "metsat%02d.txt", indice);
            logFile = SD.open(filename);
             Serial.print("fichero");
             Serial.println(indice);
             Serial.println("");
             tamano=tamano+logFile.size();
             Serial.print("tamaño: ");
             Serial.print(logFile.size());
             Serial.println(" bytes.");
             
            if (logFile) {
            while (logFile.available()) { 
      Serial.write(logFile.read());
    }
    logFile.close();
            }
          
              else 
                 {
                    Serial.println("Error al abrir el archivo");
                 }
                 indice++;
                }
                Serial.print("tamaño: ");
             Serial.print(float(tamano/1000));
             Serial.println(" Kb.");
             Serial.print("tamaño: ");
             Serial.print(tamano/1000000);
             Serial.println(" Mb.");
             Serial.print("tamaño: ");
             Serial.print(long(tamano/1000000000));
             Serial.println(" Gb.");
                dato = 0;
                Serial.println("--------------------------------------------------------------------------------------------------END1");
      }
      
   else if(dato=='2'){
    Serial.println("--------------------------------------------------------------------------------------------------");
      Serial.println("");
           Serial.println("OPCIÓN 2: BORRADO DE DATOS");
           Serial.println("");
           Serial.println("");
           Serial.println("   ***--- ATENCIÓN ---***   ");
           Serial.println("Se procederá a borrar archivos.");
           Serial.println("");
           Serial.println("j: BORRAR ARCHIVOS.");
           Serial.println("a: CANCELAR.");
           Serial.flush();
           delay(1000);
           bool eleccion = false;
                while(eleccion!= true){

                  if(Serial.available() > 0){
     
                      dato = Serial.read();
                        }
           
                       if(dato == 'j'){
                        Serial.println("--------------------------------------------------------------------------------------------------");
                            Serial.println("");
                            Serial.println("OPCIÓN j: COMIENZA BORRADO DE DATOS");
                            Serial.println("");
                            delay(3000);
                                      indice=0;
                                      while(indice<=15){
                                     sprintf(filename, "metsat%02d.txt", indice); 
                                      Serial.print(filename);
                                       Serial.println(" borrado.");
                                          logFile.close();
                                           SD.remove(filename);
                                           delay(150);
                                           indice++;
                                      } 
                                 k = 0;
                                 EEPROM.write(addr_k,k);
                                 eleccion=true;
                                 Serial.println("");
                                 Serial.println("DATOS BORRADOS");
                                 Serial.println("");
                                 delay(3000);
                                 Serial.println("--------------------------------------------------------------------------------------------------ENDj");
                             }
                        else if (dato == 'a') {
                          Serial.println("--------------------------------------------------------------------------------------------------");
                             Serial.println("");
                             Serial.println("OPCIÓN a: CANCELADO EL BORRADO DE DATOS");
                             Serial.println("");
                             delay(1000);
                             eleccion=true;
                             Serial.println("--------------------------------------------------------------------------------------------------ENDa");
                             }

                } 
                Serial.println("--------------------------------------------------------------------------------------------------END2");
      }
       else if(dato == '3'){
        Serial.println("--------------------------------------------------------------------------------------------------");
                Serial.println("");
                Serial.println("EXTRACCIÓN DE SD SEGURA");
                Serial.println("");
                dualLed(3);
                Serial.println("EXTRAER ANTES DE 20 segundos.");
                Serial.flush();
                dato=0;
                delay(20000);
                dualLed(LED_SD_CLOSE);
                Serial.println("--------------------------------------------------------------------------------------------------END3");
                  }   
                
                }
                

  void printDebug(){

   #ifdef DEBUG    // SOLO imprime si está el DEBUG activo
      Serial.print("$"); Serial.print(",");
      Serial.print(h); Serial.print(" DHTHUM, ");
      Serial.print(t); Serial.print(" DHTTEMP, ");
      Serial.print(sensor.getTempCByIndex(0)); Serial.print(" TEMPEXT, ");
      Serial.print(T,2); Serial.print(" BMPTEMP, ");
      Serial.print(P,2); Serial.print(" BMPPRES, "); 
      Serial.print(AcX); Serial.print(" AX, ");
      Serial.print(AcY); Serial.print(" AY, ");
      Serial.print(AcZ); Serial.print(" AZ, ");
      Serial.print(GyX); Serial.print(" GX, ");
      Serial.print(GyY); Serial.print(" GY, ");
      Serial.print(GyZ); Serial.println(" GZ, ");
      Serial.print(flat); Serial.print(" LAT, ");
      Serial.print(flon); Serial.print(" LON, ");
      Serial.print(ldate); Serial.print(" DATE, ");
      Serial.print(ltime); Serial.print(" TIME, ");
      Serial.print(laltitude); Serial.print(" ALTITIDE, ");
      Serial.print(lage); Serial.print(" LAGE, ");
      Serial.print(lcourse); Serial.print(" COURSE, ");
      Serial.print(lspeed); Serial.print(" SPEED, ");
      Serial.println("#");
   
   #endif

}


 void datosGPS(){
 bool newData = false;
  unsigned long chars;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
       //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {

    gps.f_get_position(&flat, &flon, &age);

    #ifdef DEBUG 
       Serial.print("LAT=");
       Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
       Serial.print(" LON=");
       Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
       Serial.print(" SAT=");
       Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
       Serial.print(" PREC=");
       Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
   #endif

    gps.get_datetime(&ldate, &ltime, &lage);
    laltitude = gps.altitude();
    lcourse = gps.course();
    lspeed = gps.speed();
    
  }
 }

unsigned long antTime = millis();

 void loop() {

  if(k>2 && k<13){
    digitalWrite(BUZZ, LOW);
     dualLed(3); 
    }
    else{
      digitalWrite(BUZZ, HIGH);
      dualLed(LED_SD_OPEN);
      delay(25);
      }
  
  while (digitalRead(PUL_2) == HIGH){
    digitalWrite(BUZZ, LOW);
    dualLed(LED_SD_CLOSE);
    lecturaSd();
    antTime = millis();
    }
    
     
   readTempHumedad(); // DHT11 Y DS18B20

   readBMP(); // BMP180

   readAccGyro(); //MPU
    digitalWrite(BUZZ,LOW);
   
   #ifdef DEBUG
     Serial.print("eeprom: ");
     Serial.println(EEPROM.read(addr_k));
  
     printDebug();
   #endif
  
   datosGPS(); // GPS
 
   //guardarDatosSd(); // SE ESCRIBEN VALORES EN SD Y SE CREAN FICHEROS SI PROCEDE
   if ((millis() - antTime) < TIME_FILE){  // cada 10 minutos cambia de indice

      sprintf(filename, "metsat%02d.txt", k);
      logFile = SD.open(filename, FILE_WRITE);
      if (logFile) { 
          printSDcard();
       }else {
          #ifdef DEBUG
            Serial.println("Error al abrir el archivo");
          #endif
       }
    }
    else {

      antTime = millis();
      k++; // incrementa el indice del fichero
      EEPROM.write(addr_k,k);
    }
   if(15 < k){    // si 15*10 = 150 minutos deja de tomar datos para proteger la SD
    dualLed(LED_SD_CLOSE);
    Serial.println("FIN DE ESCRITURAS");
    delay(1000); 
     while(1){
      digitalWrite(BUZZ,HIGH);
      lecturaSd();
       if(Serial.available() > 0){
     
                      dato = Serial.read();
                        }
           
                       if(dato == '4'){
                        Serial.println("");
                        Serial.println("REINICIANDO LOOP");
                            Serial.println("");
                            Serial.flush();
                            dato=0;
                            break;
                       }
      }
        
  }
}
