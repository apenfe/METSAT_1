
void setup()  {
  Serial.begin(9600);  }
  
void loop()  {
  Serial.print("El indice UV es: ");
  int UV_Val_RAMBAL;
  int UV;
  UV_Val_RAMBAL = analogRead(0);    
  if(UV_Val_RAMBAL < 10)  {
    Serial.println("0, nivel bajo");  }
  else  {  if(UV_Val_RAMBAL < 46)  {
    Serial.println("1, nivel bajo");  }
  else  {  if(UV_Val_RAMBAL < 65)  {
    Serial.println("2, nivel bajo");  }
  else  {  if(UV_Val_RAMBAL < 83)  {
    Serial.println("3, nivel moderado");  }
  else  {  if(UV_Val_RAMBAL < 103)  {
    Serial.println("4, nivel moderado");  }
  else  {  if(UV_Val_RAMBAL < 124)  {
    Serial.println("5, nivel moderado");  }
  else  {  if(UV_Val_RAMBAL < 142)  {
    Serial.println("6, nivel ALTO");  }
  else  {  if(UV_Val_RAMBAL < 163)  {
    Serial.println("7, nivel ALTO");  }
  else  {  if(UV_Val_RAMBAL < 180)  {
    Serial.println("8, nivel MUY ALTO");  }
  else  {  if(UV_Val_RAMBAL < 200)  {
    Serial.println("9, nivel MUY ALTO");  }
  else  {  if(UV_Val_RAMBAL < 221)  {
    Serial.println("10, nivel MUY ALTO");  }
  else  {  if(UV_Val_RAMBAL < 239)  {
    Serial.println("11, nivel Ext. ALTO 'Peligro'");  }
  else  {
    Serial.println("11+, nivel Ext. ALTO 'Peligro'");  }
  }}}}}}}}}}}
  delay(500);  }
