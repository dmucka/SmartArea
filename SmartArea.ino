/*
  Smart Area V4
  Daniel Mucka 2018
*/

//#define DEBUG

#define MAX_BUFFERSIZE 8
#define MAX_CHARSIZE 10

#include "dht.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

class InternalTimer // Objekt, ktory sluzi ako zoznam casovacov
{
  private:
    unsigned long _currentMillis;
    unsigned long * _previousMillis;
  public:
    InternalTimer(byte numberOfTimers)
    {    
      _previousMillis = new unsigned long[numberOfTimers];
      
      SetCurrentTime();
      for (byte i = 0; i < numberOfTimers; i++)
        SetPreviousTime(i);
    }
    ~InternalTimer()
    {
      delete[] _previousMillis;
    }
    void SetCurrentTime()
    {
      _currentMillis = millis();
    }
    void SetPreviousTime(byte index)
    {
      _previousMillis[index] = millis();
    }
    unsigned long GetPassedTime(byte index)
    {
      return _currentMillis - _previousMillis[index];
    }
    bool HasTimePassed(byte index, int requiredTime)
    {
      return GetPassedTime(index) >= requiredTime;
    }
};

#ifdef DEBUG
void printd(char* msg, ...)
{
  char msgBuffer[MAX_CHARSIZE];
  
  va_list argv;
  va_start(argv, msg);
  vsnprintf(msgBuffer, MAX_CHARSIZE, msg, argv);
  va_end(argv);
  
  Serial.println(msgBuffer);
  
  #ifdef RAM
  char buf[MAX_CHARSIZE];
  sprintf(buf, "ram=%d", freeRam());
  Serial.println(buf);
  #endif
}
#endif

#ifdef PRINT_DISPLAY
char msgBuffer[MAX_BUFFERSIZE][MAX_CHARSIZE];
byte msgBufferCounter = 0;

void printd(Adafruit_SSD1306 display, char* msg, ...)
{
  if (msgBufferCounter == MAX_BUFFERSIZE)
  {
    msgBufferCounter = 0;
    for (int i = 1; i < MAX_BUFFERSIZE; i++)
    {
      memset(msgBuffer[i], '\0', MAX_CHARSIZE);
    }
  }
  
	va_list argv;
	va_start(argv, msg);
  vsnprintf(msgBuffer[msgBufferCounter], MAX_CHARSIZE, msg, argv);
	va_end(argv);
	
	Serial.println(msgBuffer[msgBufferCounter]);

  display.clearDisplay();
  for (int i = 0; i < MAX_BUFFERSIZE; i++)
  {
    display.println(msgBuffer[i]);
  }
  
  display.display();
  
  #ifdef DEBUG
  char buf[MAX_CHARSIZE];
  sprintf(buf, "ram=%d", freeRam());
  Serial.println(buf);
  #endif
  
  msgBufferCounter++;
}

#endif

/* Vystupy */
#define OUT_ACCESS    8  // [GREEN LED] Vstup
#define OUT_HEATING   9  // [YELLOW LED] Ohrev vody
#define OUT_LIGHTING 10  // [WHITE LED] Osvetlenie
#define OUT_BUZZER   11  // Bzuciak
#define OUT_WARNING  12  // [RED LED] Vystrazne svetlo
#define OUT_DLED     13  // [YELLOW LED] Na arduine

/* Vstupy */
#define IN_TIMERUP   4   // Tlacidlo zvysit cas
#define IN_TIMERDOWN 3   // Tlacidlo znizit cas
#define IN_CONFIRM   2   // Tlacidlo potvrdit/prihlasit/odhlasit
#define IN_TEMP      7   // Teplotny senzor

/* Analogove vstupy */
#define ANALOG_LIGHTSENSOR 0 // Senzor svetla
#define ANALOG_RESET       4 // I2C Reset

/* Intervaly */
#define INTERVAL_DEBOUNCE  300 // [ms] Stabilizacia hodnoty detekovanej na tlacidle
#define INTERVAL_WARNING    30 // [m] Minut pred koncom sa zopne cervene svetlo Default 10
#define INTERVAL_MULTIPLIER 40 // [x] Default 60000

/* Nastavenia */
#define FREQUENCY_BUZZER 200 // [hz] Frekvencia na ktorej bude hrat bzuciak Default 2400
#define TIMER_COUNT 6        // Pocet potrebnych casovacov

/* Globalne premenne */
struct boolpack_t
{
  byte TimerUpClicked: 1;
  byte TimerDownClicked: 1;
  byte ConfirmClicked: 1;
  //byte TemperatureClicked: 1;
  byte AccessOn: 1;
  byte WarningOn: 1;
  byte BuzzerOn: 1;
} pack;

int iLightValue = 0; // Hodnota svetla
int iTemperature = 0; // Teplota
int iHumidity = 0; // Teplota
int iUserSelectedTime = 60; // Zvoleny cas zakaznika
int iCountdown = 0; // Odpocet casu zobrazeneho na displeji
InternalTimer* timer = new InternalTimer(TIMER_COUNT); // Objekt ktori posobi ako zoznam casovacov, inicializuje sa hodnotou, ktora urcuje pocet internych casovacov
Adafruit_SSD1306 display(ANALOG_RESET); // OLED displej 128x64
dht DHT; // Teplotny senzor

void setup() {
  Serial.begin(9600); // Spusti serialove pripojenie
  
  // Displej
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // 0x3C = 0x78 >> 1
  display.display(); // Zobrazi splashscreen
  delay(2000);
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(1);
  display.fillScreen(BLACK);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  
  // Nastavenie vystupov
  pinMode(OUT_ACCESS, OUTPUT);
  pinMode(OUT_HEATING, OUTPUT);
  pinMode(OUT_WARNING, OUTPUT);
  pinMode(OUT_BUZZER, OUTPUT);
  pinMode(OUT_LIGHTING, OUTPUT);
  
  // Nastavenie vstupov
  pinMode(IN_TIMERUP, INPUT_PULLUP);
  pinMode(IN_TIMERDOWN, INPUT_PULLUP);
  pinMode(IN_CONFIRM, INPUT_PULLUP);
  
  // Spusti vstavanu LED
  pinMode(OUT_DLED, OUTPUT);
  digitalWrite(OUT_DLED, HIGH);
  
  #ifdef DEBUG
  printd("init");
  #endif
}

bool bIsInverted = false;
void invert()
{ 
  if (timer->HasTimePassed(6, 1000))
  {
    timer->SetPreviousTime(6);

    if (!bIsInverted)
    {
      display.invertDisplay(true);
      bIsInverted = true;
    } else {
      display.invertDisplay(false);
      bIsInverted = false;
    }
  }
}

void render() 
{
  display.clearDisplay();
  if (pack.BuzzerOn)
    invert();
  else {
      display.invertDisplay(false);
      bIsInverted = false;
  }
  
  display.setCursor(0, 0);
  display.setTextSize(1);
  
  int percent = (iLightValue / 1024.0) * 100.0;

  display.setTextSize(2);
  display.print("Svetlo ");
  display.print(percent);
  display.setTextSize(1);
  display.print("%\n");

  display.setTextSize(2);
  display.print("\n");
  display.print("Cas ");
  if (!pack.AccessOn)
    display.print(iUserSelectedTime);
  else
    display.print(iCountdown);
  display.setTextSize(1);
  display.print("min\n");

  display.setTextSize(2);
  display.print("\n");
  display.print("T:");
  display.print(iTemperature);
  display.setTextSize(1);
  display.print((char)247);
  display.print("C ");
  display.setTextSize(2);
  display.print("H:");
  display.print(iHumidity);
  display.setTextSize(1);
  display.print("%");
  
  display.display();
}

void loop() {
  timer->SetCurrentTime(); // Nastavi aktualny cas na zaciatku kazdeho cyklu
  
  // Ziska vstup zo stlacenia tlacidiel, invertuje ich lebo sa pouziva PULL UP rezistor, ktory pri stlaceni zopne LOW signal
  pack.TimerUpClicked = !digitalRead(IN_TIMERUP);
  pack.TimerDownClicked = !digitalRead(IN_TIMERDOWN);
  pack.ConfirmClicked = !digitalRead(IN_CONFIRM);
  //pack.TemperatureClicked = !digitalRead(IN_TEMP);
  
  /* Simulacia teplotneho snimaca */
  //if (pack.TemperatureClicked) iTemperature = 30; else iTemperature = 35;
  //iTemperature = pack.TemperatureClicked ? 30 : 35;
  
  /* 
    Ak teplotny senzor nameria teplotu mensiu alebo rovnu ako 30 stupnov,
    zopne sa vyhrevne teleso, ktore bude bezat az pokial senzor nenamera teplotu viac ako 35 stupnov
  */
  if (iTemperature <= 26) digitalWrite(OUT_HEATING, HIGH);
  else if (iTemperature >= 27) digitalWrite(OUT_HEATING, LOW);
  
  /* Pred koncom zakaznikovho pobytu (10 minut pred koncom) sa zopne cervene svetlo */
  if (pack.AccessOn && !pack.WarningOn && timer->HasTimePassed(4, iUserSelectedTime * INTERVAL_MULTIPLIER - INTERVAL_WARNING * INTERVAL_MULTIPLIER))
  {
    digitalWrite(OUT_WARNING, HIGH); // Rozosvieti cervene svetlo
    
    #ifdef DEBUG
    printd("warn");
    #endif
    
    pack.WarningOn = true; // Nastavi premennu
  }
  
  /* Po skonceni celkoveho casu zakaznika sa zopne bzuciak, zhasne zelene svetlo ale cervene svetlo ostane svietit */
  if (!pack.BuzzerOn && pack.AccessOn && pack.WarningOn && timer->HasTimePassed(4, iUserSelectedTime * INTERVAL_MULTIPLIER))
  {
    digitalWrite(OUT_ACCESS, LOW); // Zhasne zelene svetlo
    
    #ifdef DEBUG
    printd("timeout");
    #endif
    
    tone(OUT_BUZZER, FREQUENCY_BUZZER); // Spusti bzuciak pri frekvencii 2400 Hz
    pack.BuzzerOn = true; // Nastavi premennu
    iCountdown = 0;
    timer->SetPreviousTime(6);
  }
    
  /* Zvysi cas pre zakaznika */
  if (!pack.AccessOn && pack.TimerUpClicked && timer->HasTimePassed(1, INTERVAL_DEBOUNCE))
  {
    timer->SetPreviousTime(1); // Zapne casovac pre aktualne tlacitko
    if (iUserSelectedTime + 30 != 390) // Zvysi cas do premennej
      iUserSelectedTime += 30;
      
    #ifdef DEBUG
    printd("up=%d", iUserSelectedTime);
    #endif
  }
  
  /* Znizi cas pre zakaznika */
  if (!pack.AccessOn && pack.TimerDownClicked && timer->HasTimePassed(2, INTERVAL_DEBOUNCE))
  {
    timer->SetPreviousTime(2); // Zapne casovac pre aktualne tlacitko
    if (iUserSelectedTime - 30 > 30) // Znizi cas do premennej
      iUserSelectedTime -= 30;
      
    #ifdef DEBUG
    printd("down=%d", iUserSelectedTime);
    #endif
  }
  
  /* Prihlasenie zakaznika do systemu */
  if (!pack.AccessOn && pack.ConfirmClicked && timer->HasTimePassed(3, INTERVAL_DEBOUNCE))
  {
    timer->SetPreviousTime(3); // Zapne casovac pre aktualne tlacitko
    digitalWrite(OUT_ACCESS, HIGH); // Zopne zelene svetlo
    pack.AccessOn = true; // Nastavi premennu
    
    #ifdef DEBUG
    printd("sel=%d", iUserSelectedTime);
    #endif
    
    timer->SetPreviousTime(4); // Zapne casovac pre zakaznikov zvoleny cas
  }
  
  /* Odhlasenie zakaznika zo systemu */
  if (pack.AccessOn && pack.ConfirmClicked && timer->HasTimePassed(3, INTERVAL_DEBOUNCE))
  {
    timer->SetPreviousTime(3); // Zapne casovac pre aktualne tlacitko
    
    digitalWrite(OUT_WARNING, LOW); // Zhasne cervene svetlo
    digitalWrite(OUT_ACCESS, LOW); // Zhasne zelene svetlo
    pack.AccessOn = false;
    pack.WarningOn = false;
    pack.BuzzerOn = false;
    noTone(OUT_BUZZER); // Vypne bzuciak
    
    #ifdef DEBUG
    printd("reset");
    #endif
  }
  
  /* Svetelny senzor + Teplotny senzor */
  if (timer->HasTimePassed(5, 1000))
  {
    timer->SetPreviousTime(5); // Nastavi casovac pre detekciu svetla na aktualny cas
    
    iLightValue = analogRead(ANALOG_LIGHTSENSOR); // Ziska analogovu hodnotu v rozmedzi 0-1024
    
    #ifdef DEBUG
    printd("l=%d", iLightValue);
    #endif
    
    if (iLightValue <= 750) digitalWrite(OUT_LIGHTING, HIGH); // Ak je hodnota svetla mensia ako 75%, zopni osvetlenie
    else digitalWrite(OUT_LIGHTING, LOW); // Inak zhasni

    DHT.read11(IN_TEMP); // Precitaj hodnoty zo senzora
    iTemperature = DHT.temperature; // Uloz teplotu do premennej
    iHumidity = DHT.humidity;
    
    #ifdef DEBUG
    printd("temp=%d", iTemperature);
    printd("humi=%d", iHumidity);
    #endif
  }

    /* Rendering */
    if (!timer->HasTimePassed(4, iUserSelectedTime * INTERVAL_MULTIPLIER))
      iCountdown = (iUserSelectedTime * INTERVAL_MULTIPLIER - timer->GetPassedTime(4)) / INTERVAL_MULTIPLIER;
      
    render();
}
