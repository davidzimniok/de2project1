/***********************************************************************
 * 
 * Use Analog-to-digital conversion to read push buttons on LCD keypad
 * shield and display it on LCD screen.
 * 
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2018 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/


/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions
#include <uart.h>         // C library. Needed for number conversions


//function protoypes
void rot_encoder();
void lcd_put_time(uint8_t row, uint8_t old[8]);
void array_cpy(uint8_t dest[8], uint8_t src[8]);
void array_sub();

//definition input/output pins
#define ENCODER_BTN PD2
#define ENCODER_PIN1 PB4
#define ENCODER_PIN2 PD3
#define ENCODER_PIN1P PORTB
#define ENCODER_PIN2P PORTD
#define ENCODER_BTNP PORTD
#define ENCODER_PIN1RP PINB
#define ENCODER_PIN2RP PIND
#define ENCODER_BTNRP PIND

//global variable for values used by functions
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

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Use Timer/Counter1 and start ADC conversion every 100 ms.
 *           When AD conversion ends, send converted value to LCD screen.
 * Returns:  none
 **********************************************************************/
int main(void){
    //set values in global structure
    data.col=15;
    data.row=0;
    data.jmovex=0;
    data.jmovey=0;
    data.semaphore=0;
    data.change=0;
    
    uint8_t clear_seq[8]={0,0,0,0,0,0,0,0};         //varibale used for delete data saved in structure  

    array_cpy(data.time_a,clear_seq);               //initialize all timers to zero
    array_cpy(data.time_min,clear_seq);
    array_cpy(data.time_stop,clear_seq);
    
    //setup input/output pins
    GPIO_mode_input_nopull(&ENCODER_PIN1P,ENCODER_PIN1);        //CLK from rotary encoder
    GPIO_mode_input_nopull(&ENCODER_PIN2P,ENCODER_PIN2);        //data from rotary encoder
    GPIO_mode_input_nopull(&ENCODER_BTNP,ENCODER_BTN);          //switch from joystick

    uart_init(UART_BAUD_SELECT(9600,F_CPU));

    //create custom character for pause and play
    uint8_t customChars[] = {0b10000,0b11000,0b11100,0b11110,0b11110,0b11100,0b11000,0b10000,
                            0b00000,0b11011,0b11011,0b11011,0b11011,0b11011,0b11011,0b00000};
    uint8_t old[8];

    // Initialize display
    lcd_init(LCD_DISP_ON_CURSOR_BLINK);
    lcd_gotoxy(1, 0); lcd_puts("ST :");
    lcd_gotoxy(1, 1); lcd_puts("MIN:");
    lcd_gotoxy(6, 0); lcd_puts("00:00:00"); 
    lcd_gotoxy(6, 1); lcd_puts("00:00:00"); 

    lcd_command(1<<LCD_CGRAM);          // Set addressing to CGRAM (Character Generator RAM) ie to individual lines of character patterns
    for (uint8_t i = 0; i < 2*8; i++)   // Copy new character patterns line by line to CGRAM
        lcd_data(customChars[i]);
    lcd_command(1<<LCD_DDRAM);          // Set addressing back to DDRAM (Display Data RAM) ie to character codes

    lcd_gotoxy(15, 0); lcd_putc(0); 
    lcd_gotoxy(15, 1); lcd_putc(0);   

    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX |= (1UL<<6);
    // Select input channel ADC0 (voltage divider pin)
    ADMUX &= ~((1UL<<0)|(1UL<<1)|(1UL<<2)|(1UL<<3));
    // Enable ADC module
    ADCSRA |= (1UL<<7);
    // Enable conversion complete interrupt
    ADCSRA |= (1UL<<3);
    // Set clock prescaler to 128
    ADCSRA |= (1UL<<0)|(1UL<<1)|(1UL<<2);

    //set falling edge interrupt
    EICRA |= (1UL<<3);

    // Configure timers and enable timer for reading data from joystick
    TIM0_overflow_16ms();
    TIM0_overflow_interrupt_enable();
    TIM1_overflow_1s();

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Infinite loop
    while (1){
        //print counter value direct to LCD when stopwatch or timer is on
        if(data.semaphore==2){
            array_cpy(old,data.time_stop);
            array_cpy(data.time_stop,data.time_a);
            lcd_put_time(0,old);
            lcd_gotoxy(data.col, data.row); 
        }
        //substract one from value setted by user and print it
        else if(data.semaphore==1){
            array_cpy(old,data.time_min);
            array_sub();
            lcd_put_time(1,old);
            lcd_gotoxy(data.col, data.row); 
        }
        
        //if stopwatch are stopped disable interrupt routine for button and print data to LCD
        if(data.semaphore==0&&data.change==1){
            EIMSK &= ~(1UL<<0);
            lcd_gotoxy(data.col, data.row);
            lcd_putc(0); 
            array_cpy(old,clear_seq);
            lcd_put_time(0,old);
            lcd_gotoxy(data.col, data.row); 
            data.change=0;
            wait_btn();
        }

        //perform action if button is pressed
        if(GPIO_read(&ENCODER_BTNRP,ENCODER_BTN)==0){
            //change number under cursor or reload stopwatch
            if(data.col!=8&&data.col!=11&&data.col!=14&&data.col!=15&&data.row==1){             
                rot_encoder();
            }
            //reset stopwatch
            if(data.col!=14&&data.col!=15&&data.row==0&&data.change==0){
                array_cpy(data.time_stop,clear_seq);
                lcd_put_time(0,clear_seq);
            }
            //start stopwatch
            if(data.col==15&&data.row==0&&data.semaphore==0){
                data.semaphore=2;
                TIM0_overflow_interrupt_disable();
                lcd_gotoxy(15, 0); 
                lcd_putc(1);
                wait_btn();
                EIMSK |= (1UL<<0);
                TIM1_overflow_interrupt_enable();          
            }
            //start timer
            if(data.col==15&&data.row==1&&data.semaphore==0) {
                data.semaphore=1;
                TIM0_overflow_interrupt_disable();
                lcd_gotoxy(15, 1); 
                lcd_putc(1); 
                wait_btn();
                EIMSK |= (1UL<<0);
                TIM1_overflow_interrupt_enable();
            }
        }

        //move up or down on LCD display
        switch(data.jmovey){
            case 1:
                if(data.row==0) data.row=1;
                lcd_gotoxy(data.col, data.row); 
                data.jmovey=0;
                data.jmovex=0;
                break;
            case 2:
                if(data.row==1) data.row=0;
                lcd_gotoxy(data.col, data.row); 
                data.jmovey=0;
                data.jmovex=0;
                break;
            default:
                break;
        }
        //move left or right on LCD display
        switch(data.jmovex){
            case 1:
                if(data.col>6) data.col--;
                lcd_gotoxy(data.col, data.row); 
                data.jmovex=0;
                data.jmovey=0;
                break;
            case 2:
                if(data.col<15) data.col++;
                lcd_gotoxy(data.col, data.row); 
                data.jmovex=0;
                data.jmovey=0;
                break;
            default:
                break;
        }
    }
    return 0;
}


/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Reads value from joystick (2 ADC channels).
 **********************************************************************/
ISR(TIMER0_OVF_vect){
    static uint8_t overfl=0;
    uint16_t val1=0,val2=0;
    overfl++;
    uint16_t i;
    if(overfl>=10&&data.semaphore==0){
        // Start ADC conversion
        ADMUX &= ~((1UL<<0)|(1UL<<1)|(1UL<<2)|(1UL<<3));
        ADCSRA |= (1UL<<6);

        //read value from ADC and decision
        val1 = ADC;

        if(val1>=900) data.jmovey=1;
        else if(val1<=100) data.jmovey=2;
        else data.jmovey=0;

        //change pin to read
        ADMUX |= (1UL<<0);
        
        //wait one conversion
        do{
            ;
        }while((ADCSRA&16)==0);
        
        //start conversion
        ADCSRA |= (1UL<<6);

        //read value from ADC and decision
        val2 = ADC;

        if(val2>=900) data.jmovex=1;
        else if(val2<=100) data.jmovex=2;
        else data.jmovex=0;

        overfl=0;
    }
    else if(overfl>=10){
        overfl=0;
    }
}

/**********************************************************************
 * Function: Timer/Counter0 overflow interrupt
 * Purpose:  Counter for timers, overflows every 1s (used prescaler in overflow routine)
 **********************************************************************/
ISR(TIMER1_OVF_vect){
    uint8_t clear_seq[8]={0,0,0,0,0,0,0,0};

    if(data.change==0) array_cpy(data.time_a,clear_seq);

    data.time_a[7]++;
    if(data.time_a[7]==10){
        data.time_a[6]++;
        data.time_a[7]=0;
    } 
    if(data.time_a[6]==6){
        data.time_a[4]++;
        data.time_a[6]=0;
    } 
    if(data.time_a[4]==10){
        data.time_a[3]++;
        data.time_a[4]=0;
    } 
    if(data.time_a[3]==6){
        data.time_a[1]++;
        data.time_a[3]=0;
    } 
    if(data.time_a[1]==10){
        data.time_a[0]++;
        data.time_a[1]=0;
    } 
    if(data.time_a[0]==10){
        array_cpy(data.time_a,clear_seq);
    } 
    //flag (value was changed)
    data.change=1;
}

/**********************************************************************
 * Function: INT0 interrupt routine
 * Purpose:  if button is pressed set semafore to zero and switch timers
 **********************************************************************/
ISR(PCINT0_vect){
    TIM1_overflow_interrupt_disable();
    data.semaphore=0;
    TIM0_overflow_interrupt_enable();
}

/**********************************************************************
 * Function: Change number under selected positon in range 0 to 9
 * Returns:  none
 **********************************************************************/
void rot_encoder(){
    uint8_t last=100,act,cnt=0,cnt_o=100;
    char str[1];

    cnt=data.time_min[data.col-6];

    //wait till the button is pressed
    wait_btn();

    last=GPIO_read(&ENCODER_PIN1RP,ENCODER_PIN1);
    do{    
        act=GPIO_read(&ENCODER_PIN1RP,ENCODER_PIN1);
        if(act!=last){
            if(GPIO_read(&ENCODER_PIN2RP,ENCODER_PIN2)!=act){
                if(data.col==6||data.col==7||data.col==10||data.col==13){
                    if(cnt<9) cnt++;
                }
                else{
                    if(cnt<5) cnt++;
                } 
            }
            else{
                if(cnt!=0) cnt--;
            }
        }
        last=act;
        //if counter value was changed print it to LCD
        if(cnt!=cnt_o){
            lcd_gotoxy(data.col, data.row);
            itoa(cnt,str,10);
            lcd_puts(str);
            cnt_o=cnt;
            data.time_min[data.col-6]=cnt;
        }
    }while(GPIO_read(&ENCODER_BTNRP,ENCODER_BTN));
    data.time_min[data.col-6]=cnt;
    lcd_gotoxy(data.col, data.row);

    //wait till button is pressed
    wait_btn();
}

/**********************************************************************
 * Function: Print time to LCD
 * Purpose:  Print only number values, characters will be the same, and print only values that was changed in previos cycles
 * Returns:  none
 **********************************************************************/
void lcd_put_time(uint8_t row, uint8_t old[8]){
    uint8_t i,temp;
    char str[1];

    for(i=0;i<8;i++){
        if(i!=2&&i!=5){
            if(row==0) temp=data.time_stop[i];
            if(row==1) temp=data.time_min[i];
            itoa(temp,str,10);
            lcd_gotoxy(i+6, row);
            if(old[i]!=temp) lcd_puts(str);
        }
    }
    lcd_gotoxy(data.col,data.row);
}

/**********************************************************************
 * Function: Copy two arrays
 * Purpose:  Copy one array to other array (to dest)
 * Returns:  none
 **********************************************************************/
void array_cpy(uint8_t dest[8], uint8_t src[8]){
    uint8_t i;
    for(i=0;i<8;i++){
        dest[i]=src[i];
    }
}

/**********************************************************************
 * Function: Substract one from value in memory
 * Returns:  none
 **********************************************************************/
void array_sub(){
    int temp1=1,temp2=0;
    uint8_t i,state=0;
    temp1=temp2-temp1;
    data.time_min[0]=(temp1-temp1%36000)/36000;
    temp1-=(temp1-temp1%36000)/36000;
    data.time_min[1]=(temp1-temp1%3600)/3600;
    temp1-=(temp1-temp1%3600)/3600;
    data.time_min[3]=(temp1-temp1%600)/600;
    temp1-=(temp1-temp1%600)/600;
    data.time_min[4]=(temp1-temp1%3600)/3600;
    temp1-=(temp1-temp1%60)/60;
    data.time_min[6]=(temp1-temp1%10)/10;
    temp1-=(temp1-temp1%10)/10;
    data.time_min[7]=temp1;
    for(i=0;i<8;i++){
        if(data.time_min[i]!=0) state=1;
    }
    if(state==0) data.semaphore=0;
}

/**********************************************************************
 * Function: Active waitig for button relase 
 * Purpose:  Prevents repeating of the same routine without purpose
 **********************************************************************/
void wait_btn(){
    //wait till the button is pressed
    do{
        ;
    }while(GPIO_read(&ENCODER_BTNRP,ENCODER_BTN)==0);
}

/**********************************************************************
 * Function: ADC complete interrupt
 * Purpose:  Display converted value on LCD screen.
 **********************************************************************/
ISR(ADC_vect){
    ;
}