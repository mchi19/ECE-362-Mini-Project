/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Fall 2016
***********************************************************************
	 	   			 		  			 		  		
 Team ID: < 09 >

 Project Name: < Bluetooth Lock >

 Team Members:

   - Team/Doc Leader: < Aseem Jha >      Signature: ______________________
   
   - Software Leader: < Max Chi >      Signature: ______________________

   - Interface Leader: < Scott Criswell >     Signature: ______________________

   - Peripheral Leader: < Aaron Barnes >    Signature: ______________________


 Academic Honesty Statement:  In signing above, we hereby certify that we 
 are the individuals who created this HC(S)12 source file and that we have
 not copied the work of any other student (past or present) while completing 
 it. We understand that if we fail to honor this agreement, we will receive 
 a grade of ZERO and be subject to possible disciplinary action.

***********************************************************************

 The objective of this Mini-Project is to .... < Develop a bluetooth-controlled deadbolt lock >


***********************************************************************

 List of project-specific success criteria (functionality that will be
 demonstrated):

 1. Ability to send pass code data to the microcontroller via bluetooth 

 2. Ability to receive messages via bluetooth terminal

 3. Ability to display time since last unlocked and status on an LCD screen

 4. Ability to lock and unlock a deadbolt lock via a servo

 5. Ability to update the passcode of the lock via bluetooth

***********************************************************************

  Date code started: <  >

  Update history (add an entry every time a significant change is made):

  Date: <  >  Name: < ? >   Update: < ? >

  Date: < ? >  Name: < ? >   Update: < ? >

  Date: < ? >  Name: < ? >   Update: < ? >


***********************************************************************
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>
#include <string.h>

/* All functions after main should be initialized here */
char inchar(void);
void outchar(char x);
void tpass_set(void);

void prints(char str[]);
void bco(char x);


//max's functions
int pass_compare(char temp_pass[4]);
void shiftout(char);
void lcdwait(void);
void send_byte(char);
void send_i(char);
void chgline(char);
void print_c(char);
void pmsglcd(char[]);
void s_tdisp(void);
void deadbolt_switch(void);
void msg_disp();

/* Variable declarations */
int inmode = 1; // 0 is 6 digit clock input mode, 1 is 4 digit password input mode
int lckflg = 1; // when 1 turns lock to locked position
int locking = 1;
int unlocking =0;
int prevlckflg = 0;
char pass[4]; // current password
char mpass[4]; // master password
char temp_pass[4]; // input from HC-05

// timer variables

int tencnt = 0;
int onecnt = 0;
int tenths = 0;
int onesec = 0;
int hrs = 0;
int min = 0;
int secns = 0;
//int timerflag = 0;
//int ampm = 0; //0 = am, 1 = pm
int tovf = 0; //flag that represents time has exceeded 24 hours
int x = 0;

int prevpb = 0;
int pbflg = 0; // pushbutton flag


//SCI VARIABLE DECLARATIONS
unsigned char rbuf[5];
unsigned char tbuf[51];
unsigned int rin = 0;
unsigned int rout = 0;
int tout = 0;
int tin = 0;

#define rsize 5
#define tsize 51

   	   			 		  			 		       

/* Special ASCII characters */
#define CR 0x0D		// ASCII return†
#define LF 0x0A		// ASCII new line†

/* LCD COMMUNICATION BIT MASKS (note - different than previous labs) */
#define RS 0x10		// RS pin mask (PTT[4])
#define RW 0x20		// R/W pin mask (PTT[5])
#define LCDCLK 0x40	// LCD EN/CLK pin mask (PTT[6])

/* LCD INSTRUCTION CHARACTERS */
#define LCDON 0x0F	// LCD initialization command
#define LCDCLR 0x01	// LCD clear display command
#define TWOLINE 0x38	// LCD 2-line enable command
#define CURMOV 0xFE	// LCD cursor move instruction
#define LINE1  0x80	// LCD line 1 cursor position
#define LINE2  0xC0	// LCD line 2 cursor position


    

	 	   		
/*	 	   		
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void) {

/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; //; disengage PLL from system
  PLLCTL = PLLCTL | 0x40; //; turn on PLL
  SYNR = 0x02;            //; set PLL multiplier
  REFDV = 0;              //; set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; //; engage PLL

/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40   ; //COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, interrupts off initially */
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
  SCICR1 =  0x00; //$9C = 156  
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port

/* Initialize peripherals */
  DDRT = 0xFF;
  ATDDIEN = 0x40;
  DDRAD = 0x00;
  
          
/* Initialize interrupts */
/* RTI INITIALIZATIONS */
/* Initialize RTI for 2.048 ms interrupt rate */	

  CRGINT_RTIE = 1;
  RTICTL = 0x48;

/* SCI INTERRUPT DRIVEN MODE initializations*/
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)
  SCICR1 = 0x00; // no parity bit 8 bit mode
  SCICR2 = 0x2C; // Transmit/receive enable, receive interrupts on, transmit interrput initally off
  
  
pass[0] = 0x32;
pass[1] = 0x33;
pass[2] = 0x34;
pass[3] = 0x35;

//temp_pass[0] = 0x32;
//temp_pass[1] = 0x33;
//temp_pass[2] = 0x34;
//temp_pass[3] = 0x35;

//////// PWM Initialization ///////
    MODRR = 0x01;
    PWME = 0x01; //check
    PWMPOL = 0x01; //check;
    PWMCTL = 0x00; //
    PWMCAE = 0x00; //left aligned
    PWMPER0 = 240; //
    PWMDTY0 = 0; // 100-190
    PWMPRCLK = 0x03; //check
    PWMSCLA = 15; //15
    PWMCLK = 0x01; //check
    

//////// TIM initiliazation ///////
    
    TSCR1 = 0x80;
    TSCR2 = 0x0C;
    TIOS = 0x80;
    TIE = 0x80;
    TC7 = 15000;

    
//////// SPI Initialization ///////
    
    PTM = 0;
    DDRM = 0xFF;
    SPICR1 = 0x50;
    SPICR2 = 0x00;
    SPIBR = 0x01;

//////// Initialize LCD////////////
    
    PTT_PTT6 = 1;
    PTT_PTT5 = 0;
    send_i(LCDON);
    send_i(TWOLINE);
    send_i(LCDCLR);
    send_i(LCDCLR);
    lcdwait();
}

	 		  			 		  		
/*	 		  			 		  		
***********************************************************************
Main
***********************************************************************
*/
void main(void) {
  	DisableInterrupts
	initializations(); 		  			 		  		
	EnableInterrupts;
	
	send_i(LCDCLR);

 for(;;) {
  
/* < start of your main loop > */

 for(x = 0; x <= 100; x++) {
  
 lcdwait();
 }
 s_tdisp();
 prevlckflg = lckflg;
 
 if (lckflg == 1){
  TIE = 0x80;
  PTT_PTT2 = 1; // red led
  PTT_PTT3 = 0; // green led
  deadbolt_switch();
 } else if (!lckflg){
  TIE = 0x00;
  PTT_PTT2 = 0;
 PTT_PTT3 = 1;
 deadbolt_switch();
 
 }
 
 if (pbflg == 1){
  pbflg = 0;
  inmode = !inmode;
  if (inmode == 0){
    prints("Enter new password");
    bco(CR);
    bco(LF);
  }
 } 
 
		prevlckflg = lckflg;
		lckflg = !pass_compare(temp_pass);
		if((prevlckflg ==0)&& lckflg){
		  locking = 1;
		} else if (prevlckflg && !lckflg){
		  unlocking = 1;
		}
		

  
   } /* loop forever */
   
}   /* do not leave main */




/*
***********************************************************************   ††††  † ††††††   †† 
 RTI interrupt service routine: RTI_ISR
************************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  	// clear RTI interrupt flagt 
  	CRGFLG = CRGFLG | 0x80; 
  
  	if(prevpb && !(PORTAD0_PTAD6)){
  	  pbflg = 1; 
  	}
  	 prevpb = PORTAD0_PTAD6;	  	
 

}

/*
***********************************************************************   ††††  † ††††††   †† 
  TIM interrupt service routine	  		
***********************************************************************
*/

interrupt 15 void TIM_ISR(void)
{
 	
 	// clear TIM CH 7 interrupt flag 
 	TFLG1 = TFLG1 | 0x80; 
    
    if(lckflg == 0 || tovf)
    {
            secns = 0;
            min = 0;
            hrs = 0;
            tovf = 0;
    }
    else // if lckflg is not set -> locked
    {
        if((lckflg == 1) && (tovf == 0)) //locked
        {
            tencnt++;
            if(tencnt == 10)
            {
                tenths = 1;
                tencnt = 0;
                onecnt++;
            }
            if(onecnt == 10)
            {
                onesec = 1;
                onecnt = 0;
                secns++;
            }
            if(secns == 60)
            {
                secns = 0;
                min++;
            }
            if(min == 60)
            {
                hrs++;
                min = 0;
            }
            if(hrs > 24)
            {
                hrs = 0;
                tovf = 1;
            }
        }
    } 
    
    
}

/*
***********************************************************************   ††††  † ††††††   †† 
  SCI interrupt service routine		 		  		
***********************************************************************
*/
// loads receive and transmit buffer using data from HC-05

interrupt 20 void SCI_ISR(void)
{
  if (SCISR1_RDRF == 1){
    
  
    /*
    if (rin + 1 % 5 == rout){
    // buffer full, insert error code here
    // output on LCD display, 
    } else {
      rbuf[rin] = SCIDRL;
      rin = rin + 1;
      rin = rin % 5; // increment rin to point to next available space
    }
  } else if (SCISR1_TDRE == 1){
    if (tin == tout){
      // transmit buffer is empty, disable transmit interrupts and exit
      SCICR2 = 0x2C;
    } else {
      SCIDRL = tbuf[tout];
      tout = tout +1 % tsize;
    }
  }
  */
   
  // enters 4 received digits as temp_pass / pass
    if (rin == 4) {
      rin = 0;
    }
    if (rin < 4){
      
    if (inmode == 1){ 
      temp_pass[rin] = SCIDRL; // combination input mode
      rin = rin + 1;
    } else if (!inmode){
      pass[rin] = SCIDRL;     // password input mode
      rin = rin + 1;
      if(rin == 4){
        prints("New password set as: ");
        bco(pass[0]);
        bco(pass[1]);
        bco(pass[2]);
        bco(pass[3]);
        inmode = 0;
      }
    }
    }
  }
    
    
      
    //    - read status register to enable TDR write
    //- check status of TBUF: if EMPTY, disable SCI transmit interrupts and exit; else, continue
    //- access character from TBUF[TOUT]
   // - output character to SCI TDR
   //- increment TOUT mod TSIZE	

  else if(SCISR1_TDRE == 1){
  if (tin == tout){
    // tbuf empty, disable interrupts and exit
    SCICR2 = 0x2C;
  } else {
    SCIDRL = tbuf[tout];
    tout = (tout + 1)%tsize;
  }
  }
}

/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 
***********************************************************************
 Name:         inchar
 Description:  inputs ASCII character from SCI serial port and returns it
 Example:      char ch1 = inchar();
***********************************************************************
*/

char inchar(void) {
  /* receives character from the terminal channel */
        while (!(SCISR1 & 0x20)); /* wait for input */
    return SCIDRL;
}


/*
***********************************************************************
 Name:         outchar    (use only for DEBUGGING purposes)
 Description:  outputs ASCII character x to SCI serial port
 Example:      outchar('x');
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}

/*
***********************************************************************
 Name:         tpass_set
 Description:  sets temp_pass variable from fifo buffer
               
 Example:      
***********************************************************************
*/
 /*
void tpass_set(void){
  int i = 0;
  int j = rin + 1; 
      j = j % 5;
  if (j == rout) {
    while (i != 5){
      temp_pass[i] = rbuf[rout];
      i = i + 1;
      rout = rout + 1;
      rout = rout % 5;
    }
  }
}
*/
int pass_compare(char temp_pass[4]){
    int checker;
		checker = strncmp(pass, temp_pass,4);
		if(checker == 0)
		{
			return TRUE; //return TRUE (1) for PWM to check
			
		}
     return 0;
}

/*
***********************************************************************   ††††   ††††  † ††††††   †† 
  SCI buffered character output routine - bco

  Places character x passed to it into TBUF

   - check TBUF status: if FULL, wait for space; else, continue
   - place character in TBUF[TIN]
   - increment TIN mod TSIZE
   - enable SCI transmit interrupts

  NOTE: DO NOT USE OUTCHAR (except for debugging)
***********************************************************************
*/

void bco(char x)
{

  while (tout == ((tin+1)%tsize));  /* wait for output buffer empty */
  tbuf[tin] = x;
  tin = (tin+1)%tsize;
  SCICR2_SCTIE = 1;  
    

}



void prints(char str[])
{
    //unsigned int length = sizeof(str)/ sizeof(char);
    int j;
    
    for( j=0; str[j] != 0; j++){
      bco(str[j]);
    }
}




/********************************************************************
 deadbolt_switch(int)
 turns the deadbolt based on PWM
 *******************************************************************/

void deadbolt_switch(void)
{
    int x =0;
    if(lckflg && locking)
    {
        PWMDTY0 = 190; // 180duty change this value to lock value
        //needs delay here
        for(x=0; x<1000; x++){
          
          lcdwait();
        }
        PWMDTY0 = 0x00;
        locking = 0;
     
    }
    else if (!lckflg && unlocking)
    {
        PWMDTY0 = 100; //100 change this value to unlock value
        //delay needed here
        for(x=0; x<1000; x++){
          
          lcdwait();
        }
        PWMDTY0 = 0x00;
        unlocking = 0;
        
    }
}



/*
 ***********************************************************************
 shiftout: Transmits the character x to external shift
 register using the SPI.  It should shift MSB first.
 MISO = PM[4]
 SCK  = PM[5]
 ***********************************************************************
 */

void shiftout(char x)

{
    
    // test the SPTEF bit: wait if 0; else, continue
    // write data x to SPI data register
    // wait for 30 cycles for SPI data to shift out
    
    int temp = 30;
    while(!SPISR_SPTEF){}
    SPIDR = x;
    while(temp != 0){
        temp--;
    }
}

/*
 ***********************************************************************
 lcdwait: Delay for approx 2 ms
 ***********************************************************************
 */

void lcdwait()
{
    int n = 5500;
    while(n){
        n--;
    }
}

/*
 ***********************************************************************
 send_byte: writes character x to the LCD
 ***********************************************************************
 */

void send_byte(char x)
{
    // shift out character
    // pulse LCD clock line low->high->low
    // wait 2 ms for LCD to process data
    
    shiftout(x);
    PTT_PTT6 = 0;
    PTT_PTT6 = 1;
    PTT_PTT6 = 0;
    lcdwait();
}

/*
 ***********************************************************************
 send_i: Sends instruction byte x to LCD
 ***********************************************************************
 */

void send_i(char x)
{
    // set the register select line low (instruction data)
    // send byte
    PTT_PTT4 = 0;
    send_byte(x);
}

/*
 ***********************************************************************
 chgline: Move LCD cursor to position x
 NOTE: Cursor positions are encoded in the LINE1/LINE2 variables
 ***********************************************************************
 */

void chgline(char x)
{
    send_i(CURMOV);
    send_i(x);
}

/*
 ***********************************************************************
 print_c: Print (single) character x on LCD
 ***********************************************************************
 */

void print_c(char x)
{
    PTT_PTT4 = 1;
    send_byte(x);
}

/*
 ***********************************************************************
 pmsglcd: print character string str[] on LCD
 ***********************************************************************
 */

void pmsglcd(char str[])
{
    int i = 0;
    while(str[i] != '\0'){
        print_c(str[i]);
        i++;
    }
}

/*
 ***********************************************
 Display for status of lock and time since last unlocked
 ***********************************************
 */

void s_tdisp()
{
    //msg_disp();
    
    int tt = 0; //temporary time variable for printing
    send_i(LCDCLR);
    chgline(0x80);   ///FIRST LINE DISPLAYS LOCK STATUS
    //pmsglcd("Status: ");
    if(lckflg == 0) //flag is not set
    {
        pmsglcd("UNLOCKED");
    }
    else //if flag is set
    {
        pmsglcd("LOCKED");
    }

    if(lckflg){
    pmsglcd(",TIME OPEN");  
    
    chgline(0xC0);    //// SECOND LINE DISPLAYS TIME SINCE PREVIOUS UNLOCK
    //prints time since last opened
    if(tovf) //24 hours is exceeded
    {
        pmsglcd("OPENED >24HR AGO");
    }
    //if(lckflg) {  
    //pmsglcd("Prev:   ");
    tt = hrs;
    print_c(tt/10 + 0x30);
    print_c(tt%10 + 0x30);
    print_c(0x3A); //prints ':'
    tt = min;
    print_c(tt/10 + 0x30);
    print_c(tt%10 + 0x30);
    print_c(0x3A); //prints ':'
    tt = secns;
    print_c(tt/10 + 0x30);
    print_c(tt%10 + 0x30);
    }
}

