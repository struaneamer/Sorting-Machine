/*
 * Final Project - Milestone 5.c
 *
 * Created: 2020-03-02 3:03:20 PM
 * Author : mech458

Course		: UVic Mechatronics 458
Milestone	: 5
Title		: Model Sorty-Ma-Jig

Name 1:	Struan Eamer	Student ID: V00873629
Name 2:	Philip Tchernov Student ID: V00764674
Name 3: Robert Noakes	Student ID: V00943839
 */
//include libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>


//Global variables
volatile unsigned int system_status;		//0 = Run; 1 = stop and move tray
volatile unsigned int ADC_result;	//10 Bit value for POT to control PWM Duty Cycles
volatile unsigned char DCOutPut;  //Initialize DC motor output
volatile unsigned int stepper_position = 0; //Initialize stepper_position to zero, 200 steps for 360
volatile unsigned int tray_position = 0;  //Initialize to zero, will home to 1. 4 states for center of 4 item types
volatile unsigned int c_output = 0x00;	//Holder for output on LEDs
volatile unsigned int st_min = 351; //Calibrated min of Steel (THESE WILL BE OVERWRITTEN BY calibration())
volatile unsigned int wh_min = 700; //Calibrated min of White
volatile unsigned int bl_min = 941; //Calibrated min of Black
volatile unsigned int highest = 0; //Temporary value used in classification
volatile unsigned int lowest = 1023; //Temporary value used in classification
volatile unsigned int calibration_count = 0;	//Used in calibration
volatile unsigned int items_in_queue = 0;	//Number of Items in Queue
volatile unsigned int pause = 0;	//0 = DC Motor On, 1 = DC Motor Off
volatile unsigned int NUMBlack = 0;	//Number of Black in Bins
volatile unsigned int NUMWhite = 0;	//Number of White in Bins
volatile unsigned int NUMSteel = 0;	//Number of Steel in Bins
volatile unsigned int NUMAlum = 0;	//Number of Alum in Bins
volatile unsigned int NUMFault = 0;	//Number of Faulty Material Codes
int stepper_array[4] = {0b00110000, 0b00000110, 0b00101000, 0b00000101};  //four steps for the stepper motor to be output from port A in sequence

/* Type definitions */
typedef struct {
	int itemCode; 	/* stores a number describing the element */
	int stage; 	/* 0: part is Processed, 1: part not processed */
} element;

typedef struct link{
	element		e;
	struct link *next;
} link;

//Declare Functions
void adc_setup();
void stepper_clockwise(int steps, int delay);
void stepper_counter_clockwise(int steps, int delay);
int stepper_moveto(int from_position, int to_position);
void motor_DC(int dc_request);
void initialize_pwm();
void initialize_interrupts();
void mTimer(int count);
void calibration( unsigned int l );
void classify( unsigned int item);
void home_stepper();

//Linked Queue Functions
void	initLink	(link **newLink);
void	link_setup	(link **h,link **t);
void 	setup		(link **h, link **t);
void 	clearQueue	(link **h, link **t);
void 	enqueue		(link **h, link **t, link **nL);
void	dequeue(link **h, link **t, link **deQueuedLink);
element firstValue	(link **h);
char 	isEmpty		(link **h);
int 	size		(link **h, link **t);

//Initialize Linked Queue pointers
	link *head;  /* The ptr to the head of the queue */
	link *tail;  /* The ptr to the tail of the queue */
	link *newLink;  /* A ptr to a link aggregate data type (struct) */
	link *rtnLink;  /* same as the above */

//Main Task
int main(void)
{
	rtnLink = NULL;  //link to return to memory initialized to NULL
	newLink = NULL;  //link to add to memory initialized to NULL

	link_setup(&head, &tail);  //setup linked queue

	TCCR1B |=_BV(CS10);  //Initialize timer (Timer Counter Control Register)

	initialize_pwm();  //start PWM at 31% DC. See function for details
	initialize_interrupts();  //Initilizes interrupts

	DDRA = 0xff;  //Set port A to output (stepper)
	DDRB = 0xff;  //Motor outputs (least 4 bits) and PWM output (pin 7)
	DDRC = 0xff;  //Debug and display on LEDs
	DDRD = 0xf0;  //Port D input (for sensors/interrupts)
	DDRF = 0x00;  //F1 for ADC conversion
	DIDR1 = 0b00000011;  //Disable digital input for 2 analog input pins
	DIDR0 = 0xff;  //Disable digital input buffer for 8 analog input pins
	adc_setup();  //Sets appropriate registries (see function)
	sei();  //Global external interrupt enable

	system_status = 0;  //initialize to sorting and conveying routine

	home_stepper();	//Rotate stepper until Home on black

	int material_code;  //0 (White Plastic), 1 (Black plastic), 2 (Aluminum), 3 (Steel)

	while(1)
	{

		switch(system_status)  {
			case(0):  {  //Conveyor state

				motor_DC(0);  //Continue counter c-clockwise

                break;
			}

			case(1):  {  //Bucket state Exit flag high so conveyor stops IF the stepper is not in the correct position yet

				motor_DC(1);						//Brake motor to Vcc, duplicated from ISR("EX") to avoid order of operations errors.

				material_code = head->e.itemCode;  //Get the material code off the item about to be dropped

				if(material_code != tray_position)  {  //If stepper motor is out of position, pause the conveyor and move the tray
					tray_position = stepper_moveto(tray_position, material_code);  //Move the stepper and update the position
				}

				dequeue(&head, &tail, &rtnLink);	//Take first-in value out of the queue. Also sets the head to the next in FIFO queue
				free(rtnLink);						//Free the now used link from memory


				switch(material_code){
					case(0):{NUMWhite++; break;}
					case(1):{NUMBlack++; break;}
					case(2):{NUMAlum++;  break;}
					case(3):{NUMSteel++; break;}
					default:{NUMFault++;}
				}

				items_in_queue--;

				system_status = 0;  //Resume conveyor and classifying operation mode

                break;
			}

			case(2): {					//System  Pause Function
				motor_DC(1);			//Break again to avoid errors

				PORTC = ( NUMBlack | 0x10 );		//Display number of sorted Black
					mTimer(1000);		//Wait 1 second
				PORTC = ( NUMSteel | 0x20 );		//Display number of sorted Steel
					mTimer(1000);		//Wait 1 second
				PORTC = ( NUMWhite | 0x40 );		//Display number of sorted White
					mTimer(1000);		//Wait 1 second
				PORTC = ( NUMAlum | 0x80 );			//Display number of sorted Alum
					mTimer(1000);		//Wait 1 second
				PORTC = ( items_in_queue | 0xf0 );	//Display items on belt
					mTimer(1000);		//Wait 1 second

				break;
			}

		} //switch
	}  //while
}  //main()

void adc_setup() {  //Function to initialize Analog/Digital conversion

	ADCSRA |= _BV(ADEN);  //Enable ADC
	ADCSRA |= _BV(ADIE);  //Enable interrupt of ADC
	ADMUX |= ( _BV(REFS0) | _BV(MUX0) );  //Set Voltage reference to VCC (Table 25-3) input on PINF1 (Table 25-4)
}

void home_stepper(){	//Sets tray to start at black
	while( tray_position != 1){	//Tray_position == 1 is black quadrant, HE and ISR(INT3_vect) will set it to one when HE trips
		stepper_clockwise(1, 20); //CW motion until HE trips
	}
	stepper_counter_clockwise(8,20);	//Correction for proper homing
}

void classify( unsigned int item ){		//Used to classify the item based off ADC values

	PORTC = item;  //Display ADC reading on PORTC
	PORTD = (item & 0x0300) >> 3;	//Bit shift and display for port D the remaining 2 bits of the 10 bit ADC result

	int sorted_item_code;		

	if(item < st_min )  {  //Aluminum (0 => 350)
		sorted_item_code = 2;	//Saves the Classified Item
	}
	else if(item < wh_min)  {  //Steel (350 => 699)
		sorted_item_code = 3;	//Saves the Classified Item
	}
	else if(item < bl_min)  {  //White Plastic (700 = 940)
		sorted_item_code = 0;	//Saves the Classified Item
	}
	else{  // Black plastic (941 =>>>)
		sorted_item_code = 1;	//Saves the Classified Item
	}

	newLink->e.itemCode = sorted_item_code;		//ItemCode in Newlink stores Material Code
	enqueue(&head, &tail, &newLink);	//Enqueues Item

	items_in_queue++;	//Keeps track of items on belt
}

void calibration( unsigned int l ){  //Calibration function for reflectivity index, NOT USED IN FINAL CODE, FROM PREVIOUS PERFORMANCE CRITERIA

	if( calibration_count < 4){  //First four are Black
		if( l < bl_min ){
			bl_min = l;
		}
	}else if( calibration_count < 8){  //Next 4 are White
		if( l < wh_min ){
			wh_min = l;
		}
	}
	else {	//Last four are steel
		if( l < st_min ){
			st_min = l;
		}
	}

	PORTC = lowest;					//Output ADC value on LEDs
	PORTD = (lowest & 0x0300) >> 3;	//Last two bits of ADC on PORTD green LEDs (5&6)
	calibration_count++;

	if (calibration_count == 12)  {  //clear the queue if the calibration run has finished
		clearQueue(&head, &tail);
		PORTC = 0x00;	//Clears port C LEDs
	}
}

ISR(ADC_vect)  //Interrupt service routine for ADC Vector (when ACD reading complete)
{
	ADC_result = ADC;  //Read the 10 bits from ADC

	if( ADC_result < lowest){	//Sets new lowest if ADC_result is lower than it.
		lowest = ADC_result;
	}

	if( ( PIND & 0x02 ) == 0x02 ){	//If object is still in front of RL, keep processing ADC (Active HIGH)
		ADCSRA |= _BV(ADSC);	//Restart ADC
	}else{
		classify( lowest ); 	//Classifies lowest ADC value
	}
}

ISR(INT0_vect)  {  //Interrupt for System Pause
	if(pause == 0){			//System is Not Paused
		motor_DC(1);		//Stop motor
		pause = 1;			//Pause Tracking variable
		system_status = 2;	//Go to display routine

	}else if(pause == 1){	//System is already paused
		pause = 0;			//Pause tracking variable
		system_status = 0;	//Resume Sorting
	}
}

ISR(INT1_vect)  { //OR sensor next to RL sensor
	initLink(&newLink);		//Creates new link
	highest = 0;	//Resets variables NOT USED IN FINAL VERSION
	lowest = 1023;	//Resets variables
	ADCSRA |= _BV(ADSC);	//Starts ADC conversion
}

ISR(INT2_vect)  {  //EX sensor at exit of conveyor belt
	motor_DC(1);  //Brake motor to Vcc
	system_status = 1;  //Set system status to 1. Once it returns to main, it will check for material/position match
}

ISR(INT3_vect){	//Hall Effect sensor
	tray_position = 1;	//Represents tray Position = Black
}

void initialize_interrupts()  {		// config the external interrupt
	EIMSK |= ( _BV(INT0) | _BV(INT1) | _BV(INT2) | _BV(INT3) ); //| _BV(INT4)) // enable interrupts 1, 2, and 3 for now
	EICRA |=  _BV(ISC01); 					//Falling edge interrupt for System Pause
	EICRA |= ( _BV(ISC11) | _BV(ISC10) );	//Rising edge for OR sensor
	EICRA |= (_BV(ISC21) & ~_BV(ISC20) ); 	//Falling edge interrupt for EX sets ISC21/0 to 1,0 per table 11-1
	EICRA |= _BV(ISC31);  					//INT3 (HE) Falling Edge Interrupt
}

void mTimer(int count){  //Timer function
	int i = 0;

	TCCR1B |=_BV(WGM12); //Bitwise comparison of Timer Counter Control Register
	OCR1A = 0x03e8;  //Set the limit to 1000
	TCNT1 = 0x000;  //Clears Timer Counter
	TIFR1 |=_BV(OCF1A);

	while(i<count){

		if((TIFR1 & 0x02) == 0x02){ //Compares flags in position 2 of TIFR1 Register

			TIFR1 |=_BV(OCF1A);  //Reset output compare match flag to 1

			i++;

		}
	}
	return;
}

void motor_DC(int dc_request)  {  //Either run or stop the DC motor depending on desired input (1 = stop, 0 = run forward)
	switch (dc_request)  {
		case 0:
			DCOutPut = 0x02;  //Run forward
			break;
		case 1:
			DCOutPut = 0x00;  //Brake to VCC
			break;
	}
	PORTB = DCOutPut;  //DC motor controller output 
}

void stepper_clockwise(int steps, int delay) {		//Stepper function for clockwise rotation
	int i;
	for(i=0;i<steps;i++){
		switch(stepper_position){
			case(0):
				PORTA = ( stepper_array[1] | stepper_array[2] ); //L1 & L3 High (1)
				mTimer(delay);
				break;

			case(1):
				PORTA = ( stepper_array[3] | stepper_array[2] ); //L2 & L3 High	(2)
				mTimer(delay);
				break;

			case(2):
				PORTA = ( stepper_array[3] | stepper_array[0] ); //L2 & L4 High (3)
				mTimer(delay);
				break;

			case(3):
				PORTA = ( stepper_array[1] | stepper_array[0] ); //L1 & L4 high (0)
				mTimer(delay);
				break;

			default:
			break;
		}//switch
		
		stepper_position = ((stepper_position+1) % 4);
		
		if( (i<13) && (delay >= 7) ){ delay = (delay-1) ;}			//Reduces delay my 1 ms at a time from 20 to max speed of 7 (20/7 = 2.86 times faster)
		if( ( (steps-i) <= 13 ) && (delay<20) ){ delay= (delay+1) ;}	//Increases delay my 1 ms at a time from 7 to 20 for end of rotation
			
	}//for
} //stepperClockwise


void stepper_counter_clockwise(int steps, int delay) {
	int i;
	for(i=0;i<steps;i++){
		switch(stepper_position){
			case(0):
				PORTA = ( stepper_array[3] | stepper_array[0] ); //L2 & L4 High (3)
				mTimer(delay);
				break;

			case(1):
				PORTA = ( stepper_array[1] | stepper_array[0] ); //L1 & L4 high (0)
				mTimer(delay);
				break;

			case(2):
				PORTA = ( stepper_array[1] | stepper_array[2] ); //L1 & L3 High (1)
				mTimer(delay);
				break;

			case(3):
				PORTA = ( stepper_array[3] | stepper_array[2] ); //L2 & L3 High	(2)
				mTimer(delay);
				break;

			default:
			break;
		}//switch
		if(stepper_position == 0)  {	//Resets to avoid negative stepper positions
			stepper_position = 4;
		}
		stepper_position--;		//Decrement stepper position. 
		
		if( (i<13) && (delay >= 7) ){ delay = (delay-1) ;}				//Reduces delay my 1 ms at a time from 20 to max speed of 7 (20/7 = 2.86 times faster)
		if( ( (steps-i) <= 13 ) && (delay<20) ){ delay= (delay+1) ;}	//Increases delay my 1 ms at a time from 7 to 20 for end of rotation
		
	}//for
} //stepper Counter Clockwise

int stepper_moveto(int from_position, int to_position)  {  //Move stepper from its current position to the centre of the desired category
	int delta = to_position - from_position;
	if(((to_position == 0) && (from_position == 1))||((to_position == 3) && (from_position == 2))||((to_position == 2) && (from_position == 3))||((to_position == 1) && (from_position == 0))){	
		stepper_clockwise(100, 20);				//Move 180 degrees (cw)
	}

	else if((delta == -2) || (delta == 1) || (delta == 3)){
		stepper_clockwise(50, 20);               //Move clockwise 90 degrees
	}

	else if((delta == 2) || (delta == -1) || (delta == -3)){
		stepper_counter_clockwise(50, 20);       //Move ccw 90 degrees
	}
	
		return to_position;

}

void initialize_pwm()  {
	TCCR1B |=_BV(CS10);  //Initialize timer (Timer Counter Control Register)
	DDRB = 0xff;   //Set Port B pin 7 to output (PWM)
	TCCR0A |= (_BV(0) | _BV(1)); //set TCCR0A  bits 0 and 1 to 1 (WGM0 and WGM1) to enable Mode 3 Fast PWM (table 13-7)
	TCCR0B &= ~(_BV(3));  //set TCCR0B Bit 3 to 0 (WGM02)

	//TIMSK0 |= _BV(1);  //Enable OCIE0A for Timer 0 (pg 115...) in the TIMK0 Use Timer/counter0 output compare with a interrupt enable. NOT REQUIRED FOR LAB4B
	TCCR0A |= _BV(7);  //Clear OC0A on Compare Match, set OC0A at TOP Table 13-2 pg 110 Bit 7,6 = 1,0
	TCCR0B |= _BV(1);  //Sets Prescaler for timer to 1/8 (Table 13-8)
	OCR0A = 0x0050; //Sets the TOP to  (80/256 = 31% duty Cycle)
}




/**************************************************************************************
* DESC: initializes the linked queue to 'NULL' status
* INPUT: the head and tail pointers by reference
*/

void link_setup(link **h,link **t){
	*h = NULL;		/* Point the head to NOTHING (NULL) */
	*t = NULL;		/* Point the tail to NOTHING (NULL) */
	return;
}/*setup*/


/**************************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error
* INPUT: the head and tail pointers by reference
*/
void initLink(link **newLink){
	//link *l;
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
	return;
}/*initLink*/


/****************************************************************************************
*  DESC: Accepts as input a new link by reference, and assigns the head and tail
*  of the queue accordingly
*  INPUT: the head and tail pointers, and a pointer to the new link that was created
*/
/* will put an item at the tail of the queue */
void enqueue(link **h, link **t, link **nL){

	if (*t != NULL){
		/* Not an empty queue */
		(*t)->next = *nL;
		*t = *nL; //(*t)->next;
	}/*if*/
	else{
		/* It's an empty Queue */
		//(*h)->next = *nL;
		//should be this
		*h = *nL;
		*t = *nL;
	}/* else */
	return;
}/*enqueue*/


/**************************************************************************************
* DESC : Removes the link from the head of the list and assigns it to deQueuedLink
* INPUT: The head and tail pointers, and a ptr 'deQueuedLink'
* 		 which the removed link will be assigned to
*/
/* This will remove the link and element within the link from the head of the queue */
void dequeue(link **h, link **t, link **deQueuedLink)  {
	/* ENTER YOUR CODE HERE */
	*deQueuedLink = *h;	// Will set to NULL if Head points to NULL
	/* Ensure it is not an empty queue */
	if (*h != NULL){

		*h = (*h)->next;

		if( *h == NULL){
			*t = NULL;	//Original Code did not check in dequeue if that item was the last in the queue. This would cause the head to be null, but the tail to point to now free memory. A wild goose chase.
		}
	}

	return;
}/*dequeue*/


/**************************************************************************************
* DESC: Peeks at the first element in the list
* INPUT: The head pointer
* RETURNS: The element contained within the queue
*/
/* This simply allows you to peek at the head element of the queue and returns a NULL pointer if empty */
element firstValue(link **h){
	return((*h)->e);
}/*firstValue*/


/**************************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
/* This clears the queue */
void clearQueue(link **h, link **t){

	link *temp;

	while (*h != NULL){
		temp = *h;
		*h=(*h)->next;
		free(temp);
	}/*while*/

	/* Last but not least set the tail to NULL */
	*t = NULL;

	return;
}/*clearQueue*/


/**************************************************************************************
* DESC: Checks to see whether the queue is empty or not
* INPUT: The head pointer
* RETURNS: 1:if the queue is empty, and 0:if the queue is NOT empty
*/
/* Check to see if the queue is empty */
char isEmpty(link **h){
	/* ENTER YOUR CODE HERE */
	return(*h == NULL);
}/*isEmpty*/


/**************************************************************************************
* DESC: Obtains the number of links in the queue
* INPUT: The head and tail pointer
* RETURNS: An integer with the number of links in the queue*/
int size(link **h, link **t){

	link 	*temp;			/* will store the link while traversing the queue */
	int 	numElements;

	numElements = 0;

	temp = *h;			/* point to the first item in the list */

	while(temp != NULL){
		numElements++;
		temp = temp->next;
	}/*while*/

	return(numElements);
}/*size*/
