//#include "LedControlMS.h"
#include "LedControl.h"
String ts = "";

LedControl lc = LedControl(51, 52, 53, 2); // Los numeros se refieren a que pin de ARDUINO tienes en cada uno de los terminales
byte *pantalla0;
byte *pantalla1;
bool flags[]  = {true, true, true, true, true, true, true, true};
int ojosActuales = 0;

//Variables encendido
int estadoPulsador=1,estadoAnteriorPulsador=1;
int presionado,flaBajada;
bool primeraVez,turnOff,encendido;
long offTimer, ledTimer, altavozTimer;
long ledTime=1000, offTime=15000;
int ledOn=9;


//Fade
bool fading = true;
int intensidad = 10;
int deltaIntensidad = -1;

//Mov lateral ojos
byte *movLatOjos[4], *movCircOjos[10];
long lastTimeOjos;
int timeOjos = 400;
int cntMLO;

//Ventilador  ojos
long lastTimeVent;
int timeVent = 250;
bool cntVent;

//DEMO
long lastTimeDemo;
int timeDemo = 8000;
int cntDemo;
bool isDemo = false;

//Parpadeo
long now, lastTimeBlink;
int timeBlink = 10;
int cntBlink;
int deltaCnt = 1;
bool visible, blinking = true;

/* 12 para el DIN, 11 para el CLK, 10 para el CS y el 1 se refiere a la asignacion de la matriz*/

//Corazón pequeño

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
byte plano[] = {
  B00000000,
  B00000000,
  B00000000,
  B11111111,
  B11111111,
  B00000000,
  B00000000,
  B00000000
};
byte completo[] = {
  B11111111,
  B11111111,
  B11111111,
  B11111111,
  B11111111,
  B11111111,
  B11111111,
  B11111111
};
byte cuadrado[] = {
  B11111111,
  B11111111,
  B11000011,
  B11000011,
  B11000011,
  B11000011,
  B11111111,
  B11111111
};
byte piramideUp[] = {
  B00011000,
  B00011000,
  B00111100,
  B00111100,
  B01111110,
  B01111110,
  B11111111,
  B11111111
};
byte piramideDown[] = {
  B11111111,
  B11111111,
  B01111110,
  B01111110,
  B00111100,
  B00111100,
  B00011000,
  B00011000
};
byte relojArena[] = {
  B11111111,
  B01111110,
  B00111100,
  B00011000,
  B00011000,
  B00111100,
  B01111110,
  B11111111
};
byte espiral[] = {
  B01111111,
  B01000001,
  B01011101,
  B01010101,
  B01000101,
  B01111101,
  B00000001,
  B11111111
};
byte aspa[] = {
  B11000011,
  B01100110,
  B00100100,
  B00011000,
  B00011000,
  B00100100,
  B01100110,
  B11000011
};
byte cruz[] = {
  B00011000,
  B00011000,
  B00011000,
  B11111111,
  B11111111,
  B00011000,
  B00011000,
  B00011000
};
byte rayo[] = {
  B00000110,
  B00001100,
  B00011100,
  B00111000,
  B01111110,
  B00001100,
  B00011000,
  B00110000
};
byte corazon[] = {
  B00000000,
  B01100110,
  B11111111,
  B11111111,
  B01111110,
  B00111100,
  B00011000,
  B00000000
};
byte intDer[] = {
  B00011100,
  B00110110,
  B00000110,
  B00011100,
  B00011000,
  B00000000,
  B00011000,
  B00011000
};
byte intIzq[] = {
  B00011000,
  B00011000,
  B00000000,
  B00011000,
  B00111000,
  B01100000,
  B01101100,
  B00111000
};
byte excIzq[] = {
  B00011000,
  B00011000,
  B00000000,
  B00011000,
  B00011000,
  B00011000,
  B00011000,
  B00011000
};
byte excDer[] = {
  B00011000,
  B00011000,
  B00011000,
  B00011000,
  B00011000,
  B00000000,
  B00011000,
  B00011000
};
byte vacio[] = {
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000
};

byte Cara_datos[] = {
  B00111100,
  B01000010,
  B10100101,
  B10000001,
  B10100101,
  B10011001,
  B01000010,
  B00111100
};

byte ojoMain[] = {
  B00111100,
  B01111110,
  B01111110,
  B01100110,
  B01100110,
  B01111110,
  B01111110,
  B00111100
};
byte ojoSleep[] = {
  B00001111,
  B00000010,
  B00000100,
  B00001111,
  B11110000,
  B00100000,
  B01000000,
  B11110000
};

byte ojoUp[] = {
  B00111100,
  B01111110,
  B01100110,
  B01100110,
  B01111110,
  B01111110,
  B01111110,
  B00111100
};
byte ojoUpIzq[] = {
  B00111100,
  B01111110,
  B01001110,
  B01001110,
  B01111110,
  B01111110,
  B01111110,
  B00111100
};
byte ojoUpDer[] = {
  B00111100,
  B01111110,
  B01110010,
  B01110010,
  B01111110,
  B01111110,
  B01111110,
  B00111100
};
byte ojoIzq[] = {
  B00111100,
  B01111110,
  B01111110,
  B01001110,
  B01001110,
  B01111110,
  B01111110,
  B00111100
};
byte ojoDer[] = {
  B00111100,
  B01111110,
  B01111110,
  B01110010,
  B01110010,
  B01111110,
  B01111110,
  B00111100
};
byte ojoDown[] = {
  B00111100,
  B01111110,
  B01111110,
  B01111110,
  B01100110,
  B01100110,
  B01111110,
  B00111100
};
byte ojoDownIzq[] = {
  B00111100,
  B01111110,
  B01111110,
  B01111110,
  B01001110,
  B01001110,
  B01111110,
  B00111100
};
byte ojoDownDer[] = {
  B00111100,
  B01111110,
  B01111110,
  B01111110,
  B01110010,
  B01110010,
  B01111110,
  B00111100
};


//----------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  // El numero que colocamos como argumento de la funcion se refiere a la direccion del decodificador

  lc.shutdown(0, false);
  lc.setIntensity(0, intensidad); // La valores estan entre 1 y 15
  lc.clearDisplay(0);
  lc.shutdown(1, false);
  lc.setIntensity(1, intensidad); // La valores estan entre 1 y 15
  lc.clearDisplay(1);
  pantalla0 = ojoMain;
  pantalla1 = ojoMain;
  movLatOjos[0] = ojoIzq;
  movLatOjos[1] = ojoMain;
  movLatOjos[2] = ojoDer;
  movLatOjos[3] = ojoMain;



  pinMode(11, OUTPUT);//Mosfet cto. encendido D8
  pinMode(12, OUTPUT);//Relé NA negado
  pinMode(9, OUTPUT);//Led interruptor MODULO MOS
  pinMode(10, INPUT_PULLUP);//Interruptor negado
  pinMode(7,OUTPUT);//MODULO MOS altavoz
  pinMode(8,INPUT);//Seta de emergencia
  //Declaramos el pin del pulsador como entrada.

  //digitalWrite11,HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(7,LOW);
  //  Serial.begin(115200);
  Serial3.begin(57600);

}

//----------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{

  now = millis();
  ///////Detección de flanco
  estadoPulsador = digitalRead(10);
 

  if (estadoPulsador != estadoAnteriorPulsador) {
    if (estadoPulsador == 0 && primeraVez == false) { //Encender
      // Serial.print(" has encendido ");
      //Serial.print("Flanco bajada");

      flaBajada = 1; //Detecta 1º Flanco de bajada
      encendido = true;
      turnOff = false;
      digitalWrite(9, HIGH);
      digitalWrite(11, HIGH);
      digitalWrite(12, HIGH);


    }
    else if (estadoPulsador == 0 && primeraVez == true) { //Apagado
      // Serial.print("Mando apagar ");
      encendido = false;
      turnOff = true;
      offTimer = now;
      ledTimer = now;
      //Serial.print("Has dado a apagar\n");
      presionado = 1;


    }
  }


  if (estadoPulsador == 1 && presionado == 1)
  {
    // Serial.print("APAGAR");
    Serial3.print('g');
    presionado = 0; //La variable vuelve a su valor original
    //digitalWrite(11,LOW); para quitar los 25s
    //digitalWrite(12,HIGH);
  }

  if (flaBajada == 1 && estadoPulsador == 1 && primeraVez == false) {
    // Serial.print("Flanco Subida");//Detecta el Flanco de subida
    flaBajada = 0;
    estadoAnteriorPulsador = estadoPulsador;
    delay(2000);
    altavozTimer=now;
    primeraVez = true; //Ha detectado los dos flancos de encendido

  }

  if (encendido == true) {
    
     if ((now - altavozTimer) > 89000) {
      digitalWrite(7, HIGH);
    //  Serial.println("Altavoz");
    }
    if(digitalRead(7)==HIGH){
       Serial3.print('s');
       Serial.print("Seta de emergencia pulsada");
    }
  }

  if (turnOff) {
    //Parpadeo del Led interruptor
    if ((now - ledTimer) > ledTime) {
      ledOn = !ledOn;
      digitalWrite(9, ledOn);
      ledTimer = now;
    }
    //Espera de 25s para que se apague el NUC y TABLET
    if (now - offTimer >= offTime) {
      // Serial.print("Apagado");
      digitalWrite(11, LOW);
      digitalWrite(12, LOW);
    }
  }

  //Comunicación con la Mega inferior

  if (Serial3.available() > 0) {
    // Serial.println("entra");
    String  ms = Serial3.readString();
    //  Serial.print("Mensaje recibido: ");
    // Serial.println(ms);
    if (ms == "en") {
      encendido = true;
    }
    if (ms == "apa")
    {
      encendido = false;
      turnOff = true;
      offTimer = now;
      ledTimer = now;
    }

    for (int i = 0; i < 8; i++) flags[i] = true;
    //Serial.print("Mensaje TS: ");
    //Serial.println(ts);
    if (ms == "fa") { //Ojo normal
      //  Serial.println("NORMALES");
      resetFade();
      isDemo = false;
      ojosActuales = 0;
      pantalla0 = ojoMain;
      pantalla1 = ojoMain;
      dibujarPantallas();
    }
    else if (ms == "fb") {  //Ojos laterales
      isDemo = false;
      resetFade();
      ojosActuales = 1;
      pantalla0 = ojoMain;
      pantalla1 = ojoMain;
      dibujarPantallas();
    }
    else if (ms == "fc") {  //Interrogaciones
      // Serial.println("interrogaciones");
      resetFade();

      isDemo = false;
      ojosActuales = 2;
      interrogacion();
      dibujarPantallas();
    }
    else if (ms == "fd") { //Corazones
      isDemo = false;
      resetFade();

      ojosActuales = 3;
      corazones();
      dibujarPantallas();
    }
    else if (ms == "fe") { //Interrogaciones
    }
    else if (ms == "fs") {
       isDemo = false;
      resetFade();

      ojosActuales = 5;
      dormir();
      dibujarPantallas();
    }
    else if (ms == "fg") {
    }
    else if (ms == "fh") {
    }
    else if (ms == "fz") { //Modo Demo
      isDemo = true;
    }
    
  }
  if (isDemo) demo();
  if (ojosActuales == 0) {
    pantalla0 = ojoMain;
    pantalla1 = ojoMain;
    if (blinking) {
      blink1();
    }
    else if (now - lastTimeBlink > 3000) {
      blinking = true;
    }
  }
  else if (ojosActuales == 1) {
    mOjosLateral();
    dibujarPantallas();
  }
  else {
    fade();
    dibujarPantallas();
  }
  //piramidesUp();
  //piramidesDown();
  //reloj();
  //mOjosLateral();
  //cuadrados();
  //fade()
  //aturdido(); //Cammbi
  //ventilador();
  //cruces();
  //aspas();
  //rayos();
  //planos();
  //corazones();
  //exclamacion();
  //interrogacion();
}
// cambio de escala entre floats
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void demo() {
  if (now - lastTimeDemo > timeDemo) {
    lastTimeDemo = now;
    if (cntDemo == 0) { //Ojos normales
      timeDemo = 10000;
      ojosActuales = 0;
      pantalla0 = ojoMain;
      pantalla1 = ojoMain;
      dibujarPantallas();
    }
    else if (cntDemo == 1) { //Ojos  laterales
      timeDemo = 11000;
      ojosActuales = 1;
      pantalla0 = ojoMain;
      pantalla1 = ojoMain;
      dibujarPantallas();
    }
    else if (cntDemo == 2) { //Interrogaciones
      timeDemo = 8000;
      ojosActuales = 2;
      interrogacion();
      dibujarPantallas();
    }
    else if (cntDemo == 3) { //Corazones
      timeDemo = 10000;
      ojosActuales = 3;
      corazones();
      dibujarPantallas();
    }
    else if (cntDemo == 4) { //Espirales
      timeDemo = 9000;
      ojosActuales = 4;
      aturdido();
      dibujarPantallas();
    }
    else if (cntDemo == 5) { //Rayos
      timeDemo = 8000;
      ojosActuales = 5;
      rayos();
      dibujarPantallas();
    }
    else if (cntDemo == 6) { //Cruces
      timeDemo = 8000;
      ojosActuales = 6;
      cruces();
      dibujarPantallas();
    }
    else if (cntDemo == 7) { //Exclamaciones
      timeDemo = 9000;
      ojosActuales = 7;
      exclamacion();
      dibujarPantallas();
    }
    else if (cntDemo == 8) { //Planos
      timeDemo = 8000;
      ojosActuales = 8;
      planos();
      dibujarPantallas();
    }
    else if (cntDemo == 9) { //Cuadrados
      timeDemo = 9000;
      ojosActuales = 9;
      cuadrados();
      dibujarPantallas();
    }
    else if (cntDemo == 10) { //Aspas
      timeDemo = 8000;
      ojosActuales = 10;
      aspas();
      dibujarPantallas();
      cntDemo = -1;
    }
    cntDemo++;
  }
}
/*
  void encender(int mLED) {
  for (int row = 0; row < 8; row++)
  {
    for (int col = 0; col < 8; col++)
    {
      lc.setLed(mLED, col, row, true); //
    }
  }
  }
  void apagar(int mLED) {
  for (int row = 0; row < 8; row++)
  {
    for (int col = 0; col < 8; col++)
    {
      lc.setLed(mLED, col, row, false); //
    }
  }
  }
*/

void RepresentarCol(byte *Datos, int matrix) //Funcion para la representacion de bytes de datos para una matriz de 8x8
{
  for (int i = 0; i < 8; i++)
  {
    if (flags[7 - i]) {
      lc.setColumn(matrix, i, Datos[7 - i]);
    }
    else {
      lc.setColumn(matrix, i, vacio[7 - i]);
    }
  }
}
void RepresentarRow(byte *Datos, int matrix) //Funcion para la representacion de bytes de datos para una matriz de 8x8
{
  for (int i = 0; i < 8; i++)
  {
    if (flags[7 - i]) {
      lc.setRow(matrix, i, Datos[7 - i]);
    }
    else {
      lc.setRow(matrix, i, vacio[7 - i]);
    }
  }
}
void dibujarPantallas() {
  RepresentarCol(pantalla0, 0);
  RepresentarCol(pantalla1, 1);
}
void dibujarPantallasRow() {
  RepresentarRow(pantalla0, 0);
  RepresentarRow(pantalla1, 1);
}
void mOjosLateral() {
  if (now - lastTimeOjos > timeOjos) {
    cntMLO++;
    if (cntMLO > 3) cntMLO = 0;
    lastTimeOjos = now;
  }
  pantalla0 = movLatOjos[cntMLO];
  pantalla1 = movLatOjos[cntMLO];
}

void blink1() {
  if (blinking) {
    flags[cntBlink] = visible;
    cntBlink += deltaCnt;
    if (cntBlink == 8) {
      deltaCnt = -1;
      visible = !visible;
      cntBlink = 7;
    }
    else if (cntBlink == -1) {
      deltaCnt = 1;
      visible = !visible;
      cntBlink = 0;
      blinking = false;
    }
    dibujarPantallas();
    lastTimeBlink = now;
  }
  //  if (blinking) {
  //    flags[cntBlink] = visible;
  //    flags[cntBlink + 1] = visible;
  //    cntBlink += deltaCnt;
  //    if (cntBlink >= 8) {
  //      deltaCnt = -2;
  //      visible = !visible;
  //      cntBlink = 7;
  //    }
  //    else if (cntBlink <= -1) {
  //      deltaCnt = 2;
  //      visible = !visible;
  //      cntBlink = 0;
  //      blinking = false;
  //    }
  //    lastTimeBlink = now;
  //  }
}

void interrogacion() {
  pantalla0 = intIzq;
  pantalla1 = intDer;
}
void exclamacion() {
  pantalla0 = excIzq;
  pantalla1 = excDer;
}
void corazones() {
  pantalla0 = corazon;
  pantalla1 = corazon;
}
void dormir(){
  pantalla0=ojoSleep;
  pantalla1=ojoSleep;
}
void planos() {
  pantalla0 = plano;
  pantalla1 = plano;
}
void rayos() {
  pantalla0 = rayo;
  pantalla1 = rayo;
}
void  cruces() {
  pantalla0 = cruz;
  pantalla1 = cruz;
}
void aspas() {
  pantalla0 = aspa;
  pantalla1 = aspa;
}
void aturdido() {
  pantalla0 = espiral;
  pantalla1 = espiral;
}
void cuadrados() {
  pantalla0 = cuadrado;
  pantalla1 = cuadrado;
}
void reloj() {
  pantalla0 = relojArena;
  pantalla1 = relojArena;
}
void piramidesUp() {
  pantalla0 = piramideUp;
  pantalla1 = piramideUp;
}
void piramidesDown() {
  pantalla0 = piramideDown;
  pantalla1 = piramideDown;
}
void ventilador() {
  if (now - lastTimeVent > timeVent) {
    cntVent = !cntVent;
    lastTimeVent = now;
  }
  if (cntVent) {
    cruces();
  }
  else {
    aspas();
  }
}

void completos() {
  pantalla0 = completo;
  pantalla1 = completo;
}
void guino() {

}

void resetFade() {
  intensidad = 15;
  lc.setIntensity(0, intensidad);
  lc.setIntensity(1, intensidad);
}

void fade() {
  if (fading) {
    intensidad += deltaIntensidad;
    if (intensidad == 0) {
      deltaIntensidad = 1;
    }
    if (intensidad == 15) {
      deltaIntensidad = -1;
      fading = true;
    }
    lc.setIntensity(0, intensidad);
    lc.setIntensity(1, intensidad);
  }
}
void scrollLateral() {
  for (int i = 0; i <= 7; i++) {
    pantalla0[i] = 2 * pantalla0[i];
  }

}
void scrollVertical() {
  pantalla0 = (pantalla0 + 1);
}
