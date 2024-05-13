/*
  GPS TRACKER MKRFOX 1200

  Toma datos de: sensor de temperatura interno y GN-701. 
  Una vez lee los datos, envia la latitud, longitud y 
  altura a la base de datos BACKEND de Sigfox y allí se
  procesan con servidores externos y otros tipos de re-
  cursos.

  Si el GPS, no es capaz de obtener una localización, 
  entonces para no desaprovechar un mensaje, se envia la 
  temperatura del procesador, de esta forma no se desapro-
  vechan los 140 mensajes diarios.

  Todos los componentes electrónicos estaán alimentados a 3.3(V) y 
  están todos conectados a una masa común.

  El GPS gn-701 está conectado a las entradas seriales del MKR1200
  estas son: 14 TX y 13 RX, de forma que el TX del GPS va conectado
  con el RX del MKR1200 y viceversa.

  Creado el 19 de agosto de 2018
  por Jgallar.
  
  modificado el 31 de enero de 2020
  por Adrián Peñalver Fernández.

  Este codigo es de dominio público.
  https://www.hackster.io/jgallar/gps-tracker-with-arduino-mkr-fox-1200-104012
  https://www.verkami.com/locale/es/projects/25687-ayudanos-a-llegar-a-la-estratosfera
*/

#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <TinyGPS.h>

#define WAITING_TIME 1     ///// bajo tiempo a 1 minuto, antes 15
#define GPS_INFO_BUFFER_SIZE 128

bool debug = true; // PONER EN FALSE ANTES DE LANZARLO

TinyGPS gps;//GPS Object

//GPS data variables
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;
char GPS_info_char;
char GPS_info_buffer[GPS_INFO_BUFFER_SIZE];
unsigned int received_char;
bool message_started = false;
int i = 0;

// GPS coordinate structure, 12 bytes size on 32 bits platforms
struct gpscoord {
  float a_latitude;  // 4 bytes
  float a_longitude; // 4 bytes
  float a_altitude;  // 4 bytes
};

float latitude  = 0.0f;
float longitude = 0.0f;
float altitud = 0;

//////////////////////////////////////////////////////////////////////// Waiting function /////////////////////////////////////////////////////////////////////////////////////
void Wait(int m, bool s) {
  //m minutes to wait
  //s slow led pulses
  if (debug) {
    Serial.print("Espere: "); Serial.print(m); Serial.println(" minutos.");
  }
  digitalWrite(LED_BUILTIN, LOW);
  if (s) {
    int seg = m * 30;
    for (int i = 0; i < seg; i++) {
      digitalWrite(LED_BUILTIN, HIGH); //LED on
      delay(500);
      digitalWrite(LED_BUILTIN, LOW); //LED off
      delay(500);
    }
  } else {
    int seg = m * 15;
    for (int i = 0; i < seg; i++) {
      digitalWrite(LED_BUILTIN, HIGH); //LED on
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW); //LED off
      delay(3000);
    }
  }
}

/////////////////// Sigfox Send Data function ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SendSigfox(String data) {
  if (debug) {
    Serial.print("Enviando... "); Serial.println(data);
    if (data.length() > 12) {
      Serial.println("Mensaje muy largo, solo los primeros 12 bytes serán enviados.");
    }
  }
  
  // Start the module
  SigFox.begin();
  // Wait at least 30mS after first configuration (100mS before)
  delay(100);
  // Clears all pending interrupts
  SigFox.status();
  delay(1);
  if (debug) SigFox.debug();
  delay(100);

  SigFox.beginPacket();
  SigFox.print(data);


  if (debug) {
    int ret = SigFox.endPacket(true);  // send buffer to SIGFOX network and wait for a response
    if (ret > 0) {
      Serial.println("No trasmisión.");
    } else {
      Serial.println("Trasmisión correcta.");
    }

    Serial.println(SigFox.status(SIGFOX));
    Serial.println(SigFox.status(ATMEL));

    if (SigFox.parsePacket()) {
      Serial.println("Respuesta desde el servidor:");
      while (SigFox.available()) {
        Serial.print("0x");
        Serial.println(SigFox.read(), HEX);
      }
    } else {
      Serial.println("No se pudo obtener respuesta del servidor.");
      Serial.println("Comprueba la cobertura Sigfox en tu area.");
      Serial.println("Si estas en interiores, acercate a una ventana.");
    }
    Serial.println();
  } else {
    SigFox.endPacket();
  }
  SigFox.end();
}


//////////////////  Convert GPS function  //////////////////
/* Converts GPS float data to Char data */

String ConvertGPSdata(const void* data, uint8_t len) {
  uint8_t* bytes = (uint8_t*)data;
  String cadena ;
  if (debug) {
    Serial.print("Tamaño: "); Serial.println(len);
  }

  for (uint8_t i = len - 1; i < len; --i) {
    if (bytes[i] < 12) {
      cadena.concat(byte(0)); // Not tested
    }
    cadena.concat(char(bytes[i]));
    if (debug) Serial.print(bytes[i], HEX);
  }

  if (debug) {
    Serial.println("");
    Serial.print("String a enviar: "); Serial.println(cadena);
  }

  return cadena;
}


////////////////////////// Get GPS position function///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
String GetGPSpositon() {

  int messages_count = 0;
  String pos;
  int segundos_espera;  /////

  if(millis() > 4980000){ 
    segundos_espera = 60; // si el tiempo de vuelo es mayor de 1h 23m de tiempo, entonces tiempo maximo de espera 60 s
    }
  else{
    segundos_espera = 120; // si el tiempo es menor de 1h 23m, entonces podemos demorar más la busqueda de satélites.
    }

  if (debug) Serial.println("GPS encendido");
  Wait(1, false);
  while (messages_count < segundos_espera) {         //2 minutos para intentar establecer conexión con el GPS
    while (Serial1.available()) {

      int GPS_info_char = Serial1.read();
      
      if (GPS_info_char == '$') messages_count ++; // start of message. Counting messages.

      if (debug) {
        if (GPS_info_char == '$') { // start of message
          message_started = true;
          received_char = 0;
        } else if (GPS_info_char == '*') { // end of message
          for (i = 0; i < received_char; i++) {
            Serial.write(GPS_info_buffer[i]); // writes the message to the PC once it has been completely received
          }
          Serial.println();
          message_started = false; // ready for the new message
        } else if (message_started == true) { // the message is already started and I got a new character
          if (received_char <= GPS_INFO_BUFFER_SIZE) { // to avoid buffer overflow
            GPS_info_buffer[received_char] = GPS_info_char;
            received_char++;
          } else { // resets everything (overflow happened)
            message_started = false;
            received_char = 0;
          }
        }
      }

      if (gps.encode(GPS_info_char)) {
        gps.f_get_position(&latitude, &longitude);
        altitud = gps.altitude() / 100; 

        // Store coordinates into dedicated structure
        gpscoord coords = {altitud, longitude, latitude};

        gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);

        if (debug) {
          Serial.println();
          Serial.println();
          Serial.print("Latitud/Longitud: ");
          Serial.print(latitude, 5);
          Serial.print(", ");
          Serial.println(longitude, 5);
          Serial.println();
          Serial.print("Fecha: "); Serial.print(day, DEC); Serial.print("/");
          Serial.print(month, DEC); Serial.print("/"); Serial.print(year);
          Serial.print(" Hora: "); Serial.print(hour, DEC); Serial.print(":");
          Serial.print(minute, DEC); Serial.print(":"); Serial.print(second, DEC);
          Serial.print("."); Serial.println(hundredths, DEC);
          Serial.print("Altitud (metros): "); Serial.println(gps.f_altitude());
          Serial.print("Rumbo (grados): "); Serial.println(gps.f_course());
          Serial.print("Velocidad(kmph): "); Serial.println(gps.f_speed_kmph());
          Serial.print("Satelites: "); Serial.println(gps.satellites());
          Serial.println();

        }

        gps.stats(&chars, &sentences, &failed_checksum);
        if (debug) Serial.println("GPS apagado");
        pos = ConvertGPSdata(&coords, sizeof(gpscoord)); //Send data
        return pos;

      }
    }
  }
  pos = "No Signal";
}

//////////////////SETUP///////////////////

void setup() {
  if (debug) {
    Serial.begin(9600);
    while (!Serial) {}// wait for serial port to connect. Needed for native USB port only
    Serial.println("Conexión serial");
  }

  //Serial1 pins 13-14 for 3.3V connection to GPS.
  Serial1.begin(9600);
  while (!Serial1) {}
  if (debug) {
    Serial.println("GPS Conectado");
  }

  if (!SigFox.begin()) {
    Serial.println("Error en el shield SIGFOX");
    return;
  }

  // Enable debug led and disable automatic deep sleep
  if (debug) {
    SigFox.debug();
  } else {
    SigFox.end(); // Send the module to the deepest sleep
  }
}

//////////////////////LOOP////////////////////////

void loop() {
  String position_data; 

  position_data = GetGPSpositon();
  
  if( position_data == "No Signal" ){
    
  SigFox.begin();
  delay(100);
  int8_t temperatura = (int)SigFox.internalTemperature();
  Serial.print("Temperatura: ");
  Serial.println(temperatura);
  SigFox.status();
  delay(1);
  SigFox.beginPacket();
  SigFox.write(temperatura);
  int resultado = SigFox.endPacket();
  SigFox.end();
    
    }
   else{
  SendSigfox(position_data);
  }
  
  if(millis() > 4980000){ // si el tiempo es mayor de... entonces acortamos el tiempo entre mensajes a un minimo de 1 minuto y un máximo de un minuto y medio.
     delay(100);
    }
    
  else{
     Wait(WAITING_TIME, false); 
      }
}
