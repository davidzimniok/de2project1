# Project 1

Application of analog joy-stick (2 ADC channels, 1 push button), rotary encoder, and Digilent PmodCLP LCD module.

## Instructions

The goal of the project is cooperation in pairs, further study of the topic, design of own solutions, implementation, creation of project documentation and presentation of results. The division of roles and tasks within the team is up to their members.

* Students work on a project in the labs during the 10th and 11th weeks of the semester.

* Through the BUT e-learning, students submit a link to the GitHub repository, which contains the complete project in PlatfomIO and a descriptive README file. The submission deadline is the day before the next laboratory, i.e. in 2 weeks.

* The source code for the AVR must be written in C and/or Assembly and must be implementable on Arduino Uno board using toolchains from the semester, ie PlatformIO and not the Arduino-style. No other development tools are allowed.

## Struktura GitHub repozitáře

   ```c
   project1        // PlatfomIO project
   ├── include         // Included files
   │   └── timer.h
   ├── lib             // Libraries
   │   └── gpio
   │      └── gpio.c
   │      └── gpio.h
   │   └── lcd
   │      └── lcd.c
   │      └── lcd.h
   │      └── lcd_definitions.h
   ├── src             // Source file(s)
   │   └── main.c
   ├── platformio.ini  // Project Configuration File
   └── README.md       // Report of this project
   ```

### Členové týmu

* Vojtěch Vídeňský (responsible for xxx)
* David Zimniok (responsible for xxx)

## Hardware description
V projektu je použit mikrokontrolér Arduino Uno založený, který je založen na AVR čipu ATMEGA328P. Jako vnější periferie jsou použity LCD displey Digilent pro zobrazování informací, rotační enkodér a joystick pro ovládání zařízení.

### Schéma zapojení
![DE2-6](https://user-images.githubusercontent.com/99399676/205460821-f3f78203-8ce1-475d-b6a6-02dcaf3acb2f.jpg)

### Digilent PmodCLP LCD module
LCD diplej je výstupní zařízení, které umožňuje zobrazení znaků ASCII kódu. Obsahuje 8 pinů pro vstupní data, 4 piny pro nastavování registrů a na piny napájení. Podrobnou dokumentaci k tomuto LCD dipleji naleznete [zde](https://digilent.com/reference/_media/pmod:pmod:pmodCLP_rm.pdf).

### Rotační Enkodér
Jedná se o typ snímače polohy, který převádí úhlovou polohu (otočení) knoflíku na výstupní signál. Dle signálu výstupního signálu se určuje na kterou stranu se enkodér otáčí.  Výstupem jsou dva obdelníkové signály, první signál je CLK a druhý  DT.  Směr otáčení se určuje vzájemné opoždění nebo předsunutí. Podrboný popis funkčnosti si můžete přečíst [zde](https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/). Obsahuje 5 pinů, dva pro napájení (+5 V, GND), jeden pro tlačítko (SW) a dva pro výstupní obdelníkové signály (CLK, DT).

V našem projektu je rotační enkodér použit k nastavování času na minutce. Stisknutím tlačítka, které rotační enkodér obsahuje, vybíráme pozici a potvrzujeme nastavenou hodnotu. Poslední funkcí je spuštění a zastavení stopek nebo minutky pomocí tlačítka na enkodéru.

### Joystick
Joystick je dvou osé vstupní zařízení, které nám umožńuje pohyb ve čtyřech směrech. Jeho konstrukce obsahuje dva potenciometry, každý pro jednu osu. Napětí na potenciometru přivádíme na analogový vstup mikrokontroléru a převádíme na digitální hodnotu. Přesný popis funkčnosti naleznete [zde](https://create.arduino.cc/projecthub/MisterBotBreak/how-to-use-a-joystick-with-serial-monitor-1f04f0). Obsahuje 5 pinů. Dva pro napájení (+5 V, GND), jeden pro talčítko (SW) a dva pro analogové hodnoty napětí z potenciometru (VRX, VRY).

Joystick používáme pro pohyb po displeji a výběru pozic kterou chceme nastavovat.

## Popis software-u

### Knihovna timer.h
Z této knihovny byly využity pouze makra pro nastavení předděliček hodinového signálu pro časovače a také makra pro povolení nebo zakázání přerušení způsobeného přetečením čítače. V kódu níže jsou z knihovny použitá makra vytažené. 

Knihovna timer.h je dostupná [zde](/include/timer.h). Popis registrů je dostupný v oficiálním manuálu na [zde](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf) na straně 84-88 pro timer 0, a na straně 108-113 pro timer 1.

```c
/** @brief Set overflow 16ms, prescaler // 101 --> 1024 */
#define TIM0_overflow_16ms()    TCCR0B &= ~(1<<CS01); TCCR0B |= (1<<CS02) | (1<<CS00);

/** @brief Enable overflow interrupt, 1 --> enable */
#define TIM0_overflow_interrupt_enable()  TIMSK0 |= (1<<TOIE0);
/** @brief Disable overflow interrupt, 0 --> disable */
#define TIM0_overflow_interrupt_disable() TIMSK0 &= ~(1<<TOIE0);


/** @brief Set overflow 1s, prescaler 100 --> 256 */
#define TIM1_overflow_1s()    TCCR1B &= ~((1<<CS11) | (1<<CS10)); TCCR1B |= (1<<CS12);

/** @brief Enable overflow interrupt, 1 --> enable */
#define TIM1_overflow_interrupt_enable()  TIMSK1 |= (1<<TOIE1);
/** @brief Disable overflow interrupt, 0 --> disable */
#define TIM1_overflow_interrupt_disable() TIMSK1 &= ~(1<<TOIE1);
```

### Knihovna gpio.h
Knihovna gpio.h obsahuje pouze deklarace funkcí, které jsou rozepsané v souboru [gpio.c](/lib/gpio/gpio.c). Z této knihovny jsou využity funkce pro nastavení vstupních pinů a pro digitální čtení stavu pinu (1/0). V obou funkcích vstupní proměnná reg, respektive ukazatel na ní nám definuje na jakém portu je připojen pin a proměnná pin definuje přesnou pozici v registru ze které se data čtou. 

```c
void GPIO_mode_input_nopull(volatile uint8_t *reg, uint8_t pin)

uint8_t GPIO_read(volatile uint8_t *reg, uint8_t pin);
```

### Library lcd.h
Knihovna použita od autora Peter Fleury. Stránky projektu jsou dostupné [zde](http://www.peterfleury.epizy.com/avr-lcd44780.html). Ke knihovně je přidán soubor [lcd_definitions.h](/lib/lcd/lcd_definitions.h), který pouze definuje podobným způsobem jako pro GPIO porty, na kterých pinech jsou zapojené datové a řídící signály pro LCD display.

### Zdrojový kód main.c
Tento soubor je rozdělen do několika částí. Nejdříve jsou definovány prototypy všech funkcí vyjma vektorů přerušení.

#### Prototypy funkcí
```c
// čtení dat z rotačního enkóderu (využívá aktivního čekání)
void rot_encoder();
// tisk časů uložených v paměti na LCD, row určuje řádek a proměnnou old se zasílá poslední vytištěná hodnota, co předchází bilání displeje
void lcd_put_time(uint8_t row, uint8_t old[8]);
// funkce pro kopírování dat z pole src do pole dest
void array_cpy(uint8_t dest[8], uint8_t src[8]);
// aktivní čekání na puštění tlačítka (zabraňuje spouštění nevyžádaných částí kódu)
void wait_btn();
// od pole dat pro minutku odečte 1 (spouštěno po každé změně - nastavení časovače na 1s)
void array_sub();
```
#### Vektory přerušení
```c
// vektor přerušení pro čtení dat z enkóderu
ISR(TIMER0_OVF_vect)
// vektor přerušení pro funkce počítání času (inkrementace vnitřního času)
ISR(TIMER1_OVF_vect)
// vektor přerušení pro vypnutí čítání - vypnutí timeru 1
ISR(PCINT0_vect)
```

#### Globální proměnné
Pro přenos dat mezi jednotlivými funkcemi a pro usnadnění celého výpočtu je vytvořená globální struktura, která nese informace o aktuálním čase (time_a), time_stop je čas na stopkách vypisovaný na LCD, time_min je uživatelem nastavená hodnota časovače na LCD, col určuje aktuální pozici na displeji v ose x, row určuje aktuální pozici na LCD v pozici y, jmovex indikuje posun v ose x (přenos stavu z přerušení do nekonečné smyčky ve funkci main), jmovey viz. jmovex pouze pro osu y, semaphore určuje v jakém stavu se aktálně nachází mikroprocesor (0 čeká na spuštění časovače, 1 funkce minutky, 2 funkce stopek) na základě těchto stavů dochází k větvení programů, change určuje průchod prvním přerušením časovače po spuštění stopek/minutek (ochrana proti nechtěnému okamžitému vypnutí časovače).

```c
struct dataframe{
    uint8_t time_stop[8];
    uint8_t time_min[8];
    uint8_t time_a[8];
    uint8_t col;
    uint8_t row;
    uint8_t jmovex;         //0 stay at place, 1 move to left, 2 move to right
    uint8_t jmovey;         //0 stay at place, 1 move up, 2 move down
    uint8_t semaphore;      //0 waiting for time set, 1 timer , 2 stopwatch
    uint8_t change;
} data;
```

#### Funkce main()
První části je inicializace mikroprocesoru a periferií (ADC, řízení přerušení) včetně nastavení globálních poměnných. Funkce while aktivně čeká na preempci časovači. Funkce while odpovídá za časově náročné operace (posuny v osách, výpisy na LCD) a ulehčuje tak vektorům přerušení. Celá funcke main je detilně popsána v následujícícm vývojovém diagramu. 

![flowchart of main function](/images/main.png)

#### Vývojové diagramy pro vektory přerušení

vektory přerušení jsou všechny definovány v [main.c](/src/main.c).

![flowchart interrupt vectors](/images/vectors.png)

#### Vývojový diagram pro funkci lcd_put_time(uint8_t row, uint8_t old[8])

![flowchart put_time function](/images/put_time.png)

#### Vývojový diagram pro funkci rot_encoder()

![flowchart rot_encoder function](/images/rot_encoder.png)

#### Vývojový diagram pro funkci void array_cpy(uint8_t dest[8], uint8_t src[8]) and wait_btn()

![flowchart wait_btn function](/images/wait_btn.png)

#### Vývojový diagram pro funkci array_sub()

![flowchart array_sub function](/images/array_sub.png)

## Video

Insert a link to a short video with your practical implementation example (1-3 minutes, e.g. on YouTube).

## Zdroje

1. Datasheet ATmega328p: [https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)
2. Peter Fleury LCD library: [http://www.peterfleury.epizy.com/avr-lcd44780.html](http://www.peterfleury.epizy.com/avr-lcd44780.html)
3. Rotary encoder reading with Arduino: [https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/](https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/)
4. Funkce v knihovně GPIO read: [https://github.com/tomas-fryza/digital-electronics-2](https://github.com/tomas-fryza/digital-electronics-2)
