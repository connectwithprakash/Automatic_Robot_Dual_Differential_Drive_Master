/*
 * zonedrive.h
 *
 * Created: 12/20/2017 5:38:00 PM
 *  Author: abheesh
 */ 


#ifndef ZONEDRIVE_H_
#define ZONEDRIVE_H_

#include <util/delay.h>
#include "drive.h"
#include "encoder.h"
#include "headers.h"
#include "Flags.h"
#include "retry.h"

#define SHUTTLECOCK_STATUSPORT		PINL
#define SHUTTLECOCK_STATUSPIN		PINL6	
#define ZONE_STATUSPORT				PINL
#define ZONE_STATUSPIN				PINL4
#define RACK_STATUSPORT				PINL


#define RACK_STATUSPIN				PINL2

#define SHUTTLECOCKPIN				L,6
//zone pin suggests IR pin which detects manual robot is in front of automatic robot or not 
#define ZONEPIN						L,4
#define RACKPIN						L,2


/////////////////////////////////////////
/*these flags says whether robot has shuttlecock or golden rack*/

bool ShuttleCockGiven = false;
bool GoldenRackGiven = false;
bool ShuttleCockArmGone = false;


////////////////////////////////////////
/*these flags says robot where to go to throw shuttlecock*/
bool _b_Transmit_once = false;
bool GoThrowingZone1 = false;
bool GoThrowingZone2 = false;
bool GoThrowingZone3 = false;

/////////////////////////////////////////
/*these are flags says automatic robot goto specific place for loading of shuttlecock*/
bool backtoLZ1 = false;
bool backtoLZ2 = false;
bool gotoLZ2 = false;
bool ManualInFrontOfLZ2 = true;
bool sendoncetorotateGeneva = true;
bool check_stable_robot = false;

////////////////////////////////////////

/*these variable says where the robot is right now
and 'moving' and 'notmoving' are locomotive state of robot*/
enum {
	inStart_point,
	inFirstloadingCorner,
	inTZ1,
	inTZ2,
	inTZ3,
	inLZ1,
	inLZ2,
	moving,
	notmoving
	};

/////////////////////////////////////////
/*these are constant coordinates of throwing positions*/
struct coordinates{
	int x;
	int y;
	};

const struct coordinates Throwingzone1 = {4600,1950};	//4700 -x 1900 -y
const struct coordinates Throwingzone2 = {6800,1950}; 
const struct coordinates Throwingzone3 = {6500,5350};

/////////////////////////////////////////
/*these are state and position of robot in start zone*/
volatile unsigned int where = inStart_point;
volatile unsigned int robotState = notmoving;


/* The whole Game play of robot is divided into 9 different tasks which are as-*/

/*
task1 - find junction at X so that it could turn to Y to head towards LZ1
task2 - find junction at Y for loading of shuttlecock
task3 - find next junction at TZ1 and throw shuttlecock from there
task4 - come to loading zone 1 from TZ1
task5 - goto loading zone 2 from loading zone 1
task6 - goto throwing zone 2 from loading zone 2
task7 - goto loading zone 2 from throwing zone 2
task8 - goto throwing zone 3 from loading zone 2

*/
/*at start zone none of tasks are done*/
bool task1 = false;
bool task2 = false;
bool task3 = false;
bool task4 = false;
bool task5 = false;
bool task6 = false;
bool task7 = false;
bool task8 = false;
bool task9 = false;
bool xJunctionMeetFromTZ2 = false;

//this indicates main switch on
extern bool mainSwitchOn;

//this indicates godirectly to loading zone 2 and after that normal game
extern bool directlyLZ2;

//this indicates godirectly to tz3 and throw
extern bool directlyTZ3;

//this indicates goto tz3 from lz2 directly
extern bool LZ2ForTZ3;

//this indicates always throw from tz2
extern bool alwaysTZ2;

//this indicates always throw from tz1
extern bool alwaysTZ1;

//this indicates normal game
extern bool normalGame;

//////////////////////////
void updateZoneflag();

char receiveAck;
/////////////////////////

void gorockthegamefield(void)
{
	
	//light up led's//
// 	if((RACK_STATUSPORT & (1<<RACK_STATUSPIN))){
// 		PORTK |= (1<<PK3);
// 	}
// 	else{
// 		PORTK &= ~(1<<PK3);
// 	}
// 	if((ZONE_STATUSPORT & (1<<ZONE_STATUSPIN))){
// 		PORTK &= ~(1<<PK2);
// 	}
// 	else{
// 		PORTK |= (1<<PK2);
// 	}
	
	
	if((where == inLZ1 || where == inLZ2) && robotState == notmoving){
		/*if the robot is in loading zone 1 after completing task3 and task4 
		but yet to complete task5*/ 
		if(task4 && !task5){
			/*if there is no manual robot infront of automatic robot*/
			if((ZONE_STATUSPORT & (1<<ZONE_STATUSPIN)) && !alwaysTZ1){
				/*go directly to loading zone 2 and manual robot is waiting there*/
				where = inLZ1;
				gotoLZ2 = true;
				ShuttleCockGiven = true;	
				ShuttleCockArmGone = true;
			}
			
			/*if there is manual robot infront of automatic robot*/
			
			else{
				/*if there is manual robot ahead of automatic robot then goto throwing zone1*/
				GoThrowingZone1 = true;
				task3 = task4 = false;
				updateZoneflag();
			}	
		}
 		else if(task6 && task7 ){
			 //if manual robot is not ahead and not always throw from throwing zone 2
			 //or just goto throwing zone 3 from loading zone 2		
 			if( ((ZONE_STATUSPORT & (1<< ZONE_STATUSPIN)) && !alwaysTZ2) || LZ2ForTZ3){
			    /*//uart0_puts("no manual in front\r\n");*/
 				where = inLZ2;
 				ManualInFrontOfLZ2 = false;
 				updateZoneflag();
 				Hold_Position();
 			}
			//if there is manual robot ahead of automatic robot && robot is not going 
			//..directly to throwing zone 3 
 			else if (!(ZONE_STATUSPORT & (1<< ZONE_STATUSPIN)) && !LZ2ForTZ3){
				/*//uart0_puts("manual infront \r\n");*/
				ManualInFrontOfLZ2 = true;
 				xJunctionMeetFromTZ2 = false;
				updateZoneflag();
				Hold_Position();
 			}
			//if always throw from throwing zone 2
			//automatic robot must force to know there is manual robot ahead.
			else if(alwaysTZ2){
				/*//uart0_puts("always tz2 \r\n");*/
				ManualInFrontOfLZ2 = true;
				updateZoneflag();
				Hold_Position();
			}
			//otherwise updatezone flag
 			else{
 				updateZoneflag();
 				Hold_Position();
 			}
 		}
		/*if the robot is in loading zone 1 or loading zone 2  otherwise */
		else{
			updateZoneflag();
			Hold_Position();
		}
	}
	
	
	////move from start zone to corner of loading zone
	if(!task1 && where == inStart_point && (normalGame||alwaysTZ1)){	
		compass.setPid(2.0,0,30);
		////uart0_puts("hello\r\n");
		movx(Throwingzone1.x,Front,STARTZONEtoCORNER);
		//movDegree(8);
		robotState = moving;
		////uart0_puts("going ahead \t");
		if(abs(encoderX.getdistance()) >= 4400){
			encoderY.resetCount();
			linetrackerXjunctionWatch();
			////uart0_puts("int on");
		}
		////uart0_puts("\r\n");
	}
	else if((directlyLZ2 || directlyTZ3 || LZ2ForTZ3 || alwaysTZ2) && !task1){
		movx((Throwingzone2.x-150),Front,STARTZONEtoCORNER);
		robotState = moving;
		if(abs(encoderX.getdistance()) >= 6600){
			////uart0_puts("interrupt on\t");
			linetrackerXjunctionWatch();
		}
	}
	///move from corner to loading zone1 if task1 is completed and task2 not completed
	else if(task1 && !task2){	
		where = inFirstloadingCorner;
		startingAtFront = false;
		compass.setPid(2.0,0,30);
		////uart0_puts("moving aheead \r\n");
		robotState = moving;
		linetrackerXjunctionWatchOff();
		linetrackerYjunctionWatch();
		//movYForwardSlow(CORNERtoLZ1);
		Move_Yaxis_Slow(1000, Front, CORNERtoLZ1);
		//movYForwardSlow(CORNERtoLZ1);
	}
	/*if task2 is completed and robot just reached loading zone 1*/
	else if(task1 && task2 && where == inFirstloadingCorner && (robotState == moving) && (normalGame||alwaysTZ1)){
		where = inLZ1;
		compass.SETPOINT = getYawGY88();
		////uart0_puts("in loading zone 1\r\n");
		//compass.setPid(2.0,0,30);
		robotState = notmoving;
		linetrackerYjunctionWatchOff();
		BrakeMotor();
		Hold_Position();
		encoderX.resetCount();
		encoderY.resetCount();
	}
	else if(where == inFirstloadingCorner && task2 && (directlyLZ2 || directlyTZ3 || LZ2ForTZ3 || alwaysTZ2) && robotState == moving){
		linetrackerYjunctionWatchOff();
		//uart3_putc('h');
		startingAtFront = false;
		where = inLZ2;
		robotState = notmoving;
		BrakeMotor();
		encoderX.resetCount();
		encoderY.resetCount();
		if(directlyLZ2){
			task1 = task2 = task3 = task4 = task5 = true;
			ManualInFrontOfLZ2 = true;
		}
		else if(directlyTZ3){
			task1 = task2 = task3 = task4 = task5 = task6 = task7 = true;
			ShuttleCockArmGone = true;
			ShuttleCockGiven = true;
			GoThrowingZone3 = true;
			uart3_putc('j');
		}
		else if(LZ2ForTZ3){
			task1 = task2 = task3 = task4 = task5 = task6 = task7 = true;
			ManualInFrontOfLZ2 = false;
		}
		else if(alwaysTZ2){
			task1 = task2 = task3 = task4 = task5 = true;
			ManualInFrontOfLZ2 = true;
		}
	}
	/*if Shuttlecock is given*/
	if(ShuttleCockGiven && ShuttleCockArmGone)
	{
		/*if manual robot is ahead of automatic robot and automatic robot
		has not completed task3*/
			if(GoThrowingZone1 && !task3 && where == inLZ1){
				robotState = moving;
				Move_Yaxis(Throwingzone1.y+50,Front,LZ1toTZ1);
				//movy(Throwingzone1.y,Front,LZ1toTZ1);
				//uart0_puts("going tz1\t");
				if(abs(encoderY.getdistance()) >= 1600){
					linetrackerYjunctionWatch();
					////uart0_puts("INT ON");
				}
				////uart0_puts("\r\n");
			}
			/* if task3 is completed and robot just reached throwingzone 1 then*/
			else if(task3 && !task4 && where == inLZ1 && robotState == moving){
				linetrackerYjunctionWatchOff();
				////uart0_puts("reached throwing zone 1\r\n");
				BrakeMotor();
				//uart3_putc('1');
				where = inTZ1;
				encoderX.resetCount();
				encoderY.resetCount();
				robotState = notmoving;
				
			}
			/* if in throwing zone 1 and robot is notmoving then hold this position and wait till throwing
			   mechanism acknowledges throwing*/
			if(where == inTZ1 && robotState == notmoving){
			//	//uart0_puts("Throwing \r\n");
				Hold_Position();

				if(_b_Transmit_once)	//Stable_Robot() && 
				{	
					uart3_putc('1');
					//uart3_puts("Job Done\r\n\n");
					_b_Transmit_once = false;
				}
				
				receiveAck = uart3_getc();
				if(receiveAck == 'g'){
					////uart0_puts("received ack\r\n");
					backtoLZ1 = true;
					GoThrowingZone1 = false;
					receiveAck = ' ';
				}
			}
			/*if acknowledge received from throwing mechanism after throwing then back to loading zone 2*/
			if(backtoLZ1 && task3 && !task4){
				////compass.setPid(6.6,0.02,10.0);
				////uart0_puts("Returning from tz1 \t");
				Move_Yaxis(Throwingzone1.y-50, Back, LZ1toTZ1);
				//movy(Throwingzone1.y, Back,TZ1toLZ1);
				robotState = moving;
				if(abs(encoderY.getdistance()) >= 1200){
					linetrackerYjunctionWatch();
					////uart0_puts("interrupt on");
				}
				////uart0_puts("\r\n");
			
			}
			/*if after returning from throwing zone1, junction on line is detected i.e loading zone1 
			then stop and wait for communication*/
			else if(task4 && robotState == moving && where == inTZ1){
				////uart0_puts("REached loading zone 1\r\n");
				linetrackerYjunctionWatchOff();
				where = inLZ1;
				BrakeMotor();
				encoderX.resetCount();
				encoderY.resetCount();
				robotState = notmoving;
				//this statement below determines automatic robot is not moving and waiting for shuttlecock loading;
				ShuttleCockGiven = false;
				ShuttleCockArmGone = false;
				backtoLZ1 = false;
			}
			/*if there is no manual robot ahead of automatic robot go to loading zone 2*/
			else if(gotoLZ2 && !task5){
				////uart0_puts("heading loading zone 2\t");
				//compass.setPid(6.6,0.02,10.0);
				movx(2100,Front,LZ1toLZ2);
				robotState = moving;
				if(abs(encoderX.getdistance()) >= 1900){
					////uart0_puts("interrupt on");
					linetrackerXjunctionWatch();
				}
				////uart0_puts("\r\n");
			
			}
			/*if X junction near loading zone 2 is detected and robot was previously on loading zone 1*/
			else if(task5 && robotState == moving && where == inLZ1){
				////uart0_puts("reached loading zone 2\r\n");
				gotoLZ2 = false;
				linetrackerXjunctionWatchOff();
				BrakeMotor();
				where = inLZ2;
				robotState = notmoving;
				encoderX.resetCount();
				encoderY.resetCount();
				//this statement below determines automatic robot is not moving and waiting for shuttlecock loading;
				ShuttleCockGiven = false;
				ShuttleCockArmGone = false;
			}
			
			
			/*if there is manual robot ahead of automatic robot && golden rack is not given and shuttlecock
			is given*/
			if(GoThrowingZone2 && !task6 ){
				////uart0_puts("going tz2 \t");
				//compass.setPid(6.6,0.02,10.0);
				robotState = moving;
				Move_Yaxis(Throwingzone2.y+40, Front, LZ2toTZ2);
				//movy(Throwingzone2.y, Front,LZ2toTZ2);
			
				if(abs(encoderY.getdistance()) >=1200){
				////uart0_puts("interrupt on");
					linetrackerYjunctionWatch();
				}
				////uart0_puts("\r\n");

			}
			/*if robot just reached throwingzone 2 */
			else if(task6 && !task7 && where == inLZ2 && robotState == moving){
				////uart0_puts("Reached tz2 \r\n");
				where = inTZ2;
				linetrackerYjunctionWatchOff();
				encoderX.resetCount();
				encoderY.resetCount();
				robotState = notmoving;
				BrakeMotor();
				//give command to throwing mechanism to throw.
				//uart3_putc('2');
				
			}
			/* if in throwing zone 2 and robot is notmoving then hold this position and wait till throwing
			   mechanism acknowledges throwing*/
			if(where == inTZ2 && robotState == notmoving){
// 				if (!check_stable_robot)
// 				{
// 					check_stable_robot = Stable_Robot();
// 				}
				velocity_robot[0] =0;
				velocity_robot[1]=0;
				Hold_Position();
				if(/*check_stable_robot &&Goto_Fence_And_Detect() &&*/ _b_Transmit_once)	//Stable_Robot() &&
				{
					uart3_putc('2');
					encoderX.resetCount();
					encoderY.resetCount();
					_b_Transmit_once = false;
				}
				
				receiveAck = uart3_getc();
				if(receiveAck == 'g'){
					backtoLZ2 = true;
					GoThrowingZone3 = false;
					receiveAck = ' ';
				}
			}
			///if acknowledge received from throwing mechanism after throwing then back to loading zone 2
			if(backtoLZ2 && task6 && !task7){
				////uart0_puts("returning to loading zone 2 \t");
				//compass.setPid(6.6,0.02,10.0);
				//if(!xJunctionMeetFromTZ2){
					//movx(550,Front,40);
					//robotState = moving;
					//if(abs(encoderX.getdistance()) >= 400){
						//linetrackerXjunctionWatch();
					//}
				//}
				//else{
					Move_Yaxis(Throwingzone2.y-80, Back, LZ2toTZ2);
					//movy(Throwingzone2.y,Back,TZ2toLZ2);
					robotState = moving;
			
					if(abs(encoderY.getdistance()) >= 1200){
						////uart0_puts("int on");
						linetrackerYjunctionWatch();
						
					}
				//}
				////uart0_puts("\r\n");
				
			}
			/*if after returning from throwing zone2 junction on line is detected then stop and wait for
			communication*/
			else if(task7 && robotState == moving && where == inTZ2 ){
				Hold_Position();
				////uart0_puts("reached loading zone 2 \r\n");
				linetrackerYjunctionWatchOff();
				BrakeMotor();
				encoderX.resetCount();
				encoderY.resetCount();
				where = inLZ2;
				robotState = notmoving;
				//this statement below determines robot is not moving and waiting for shuttlecock loading;
				ShuttleCockGiven = false;
				ShuttleCockArmGone = false;
				backtoLZ2 = false;
			}
			
			/*if golden rack is given to automatic robot and says goto throwingzone 1*/
			if(GoThrowingZone3 && !task8){
				compass.setPid(2.0,0,30);//2.0
				//////uart0_puts("going tz3 \t");
				//compass.SETPOINT = 181;
				Move_Yaxis(Throwingzone3.y-125, Front, LZ2toTZ3);
				//movy(5300,Front,LZ2toTZ3);
				robotState = moving;
				if(abs(encoderY.getdistance()) >= 4000){
					linetrackerYjunctionWatch();	
					//////uart0_puts("interrupt on");
				}
				////uart0_puts("\r\n");
			}
			/*if throwing zone 3 has just reached */
			else if(task8 && where == inLZ2 && robotState == moving){
				////uart0_puts("reached tz3\r\n");
				linetrackerYjunctionWatchOff();
				BrakeMotor();
				encoderX.resetCount();
				encoderY.resetCount();
				robotState = notmoving;
				GoThrowingZone3 = false;
				where = inTZ3;
				//give command to throwing mechanism to throw.
				//uart3_putc('3');
			}
			/* if in throwing zone 3 and robot is notmoving then hold this position and wait till throwing
			   mechanism acknowledges throwing*/
			else if(task8 && where == inTZ3 && robotState == notmoving){
				//uart3_puts("\r\n\ninside\r\n\n");
				//velocity_robot[0] = velocity_robot[1] = velocity_robot[2] = 0;
// 				if (!check_stable_robot)
// 				{
// 					check_stable_robot = Stable_Robot();
// 				}
				if(Goto_Fence_And_Detect() && _b_Transmit_once )	//Stable_Robot() && 
				{	
					uart3_putc('3');
					_b_Transmit_once = false;
				}
				
				receiveAck = uart3_getc();
				
				if (receiveAck == 'p')
				{
					pressRobot = true;
				}
				else if (receiveAck == 'd')
				{
					pressRobot = false;
				}
				
				if(receiveAck == 'g'){
					backtoLZ2 = true;
					GoThrowingZone3 = false;
					receiveAck = ' ';
				}
			}
			/*if acknowledge received from throwing mechanism after throwing then back to loading zone 2*/
			if(backtoLZ2 && task8 && !task9){
				//////uart0_puts("back to lz2\t");
				compass.setPid(2.0,0,30);//2.0
					//movy(5300,Back,LZ2toTZ3);
					robotState = moving;
					if(abs(encoderY.getdistance()) >= 4000){
						//////uart0_puts("interrupt on");
						linetrackerYjunctionWatch();
					}
				//////uart0_puts("\r\n");
			}
			/*if after returning from throwing zone3 junction on line is detected then stop and wait for
			communication*/
			else if(task9 && robotState == moving && where == inTZ3 ){
				//////uart0_puts("reached loading zone 2\r\n");
				linetrackerYjunctionWatchOff();
				BrakeMotor();
				encoderX.resetCount();
				encoderY.resetCount();
				where = inLZ2;
				robotState = notmoving;
				//this statement below determines robot is not moving and waiting for shuttlecock loading;
				ShuttleCockGiven = false;
				ShuttleCockArmGone = false;
				backtoLZ2 = false;
			}
	}
//  	 	if(task1)	//uart0_puts("1 \t");
//  	 	else        //uart0_puts("0 \t");
//  	 	if(task2)	//uart0_puts("2 \t");
//  	 	else        //uart0_puts("0 \t");
//  	 	if(task3)	//uart0_puts("3 \t");
//  	 	else        //uart0_puts("0 \t");
//  	 	if(task4)	//uart0_puts("4 \t");
//  	 	else        //uart0_puts("0 \t");
//  	 	if(task5)	//uart0_puts("5 \t");
//  	 	else        //uart0_puts("0 \t");
//  	 	if(task6)	//uart0_puts("6 \t");
//  	 	else        //uart0_puts("0 \t");
//  	 	if(task7)	//uart0_puts("7 \t");
//  	 	else        //uart0_puts("0 \t");
//  		if(task8)	//uart0_puts("8 \n");
//  		else        //uart0_puts("0 \n");

}

void updateZoneflag(void){
	/*if low on shuttlecock pin then shuttlecock received
	i.e if manual robot arm is extended to give shuttlecock*/
	if(!ShuttleCockGiven){
		////uart0_puts("entered above \r\n");
		if(!(PINL & (1<<PL6)) &&  where == inLZ1 ){
			//'w' is sent to throwing mechanism to grip shuttlecock
			//uart0_puts("shuttlecock given \r\n");
			uart3_putc('o');
			ShuttleCockGiven = true;
			//certain delay is needed so that robot first grabs shuttlecock and moves//
			GoThrowingZone1 = true;
			task3 = task4 = false;
		}
		/*if low on shuttlecock pin then shuttlecock received
		i.e if manual robot arm is extended to give shuttlecock*/
		else if(!(SHUTTLECOCK_STATUSPORT & (1<<SHUTTLECOCK_STATUSPIN)) &&  where == inLZ2 && ManualInFrontOfLZ2){
		/*	//uart0_puts("Shuttlecock given in loading zone 2\r\nManual robot ahead \r\n");*/
			//'o' is sent to throwing mechanism to grip shuttlecock
			uart3_putc('o');
			ShuttleCockGiven = true;
			//certain delay is needed so that robot first grabs shuttlecock and moves//
			GoThrowingZone1 = false;
			GoThrowingZone2 = true;
			//robotState = moving;
			task3 = task4 = task5 = true;
			task6 = task7 = false;
		}
		//if manual robot arm is not extended	
		else{
			/*//uart0_puts("shuttlecock not given \r\n"); */
			ShuttleCockGiven = false;
		}
	}
	//if shuttlecock given and arm is gone send 'w' to throwing mechanism to give to gripper
	//and move robot
	if(ShuttleCockGiven && (PINL & (1<<PL6)) &&  (where == inLZ1 || where == inLZ2) ){
		/*//uart0_puts("Shuttlecock arm gone \r\n");*/
		ShuttleCockArmGone = true;
		uart3_putc('w');
	}
	
	/*if low on golden rack pin then rack is received
	i.e if rack is received above geneva and robot is in loading zone 2*/
	if(!ShuttleCockGiven || !GoldenRackGiven){
		if(sendoncetorotateGeneva && !(PINL & (1<<PL6)) && ((RACK_STATUSPORT & (1<<RACK_STATUSPIN))) && !ManualInFrontOfLZ2 && where == inLZ2){
			uart3_putc('k');
			sendoncetorotateGeneva = false;
		}
		////uart0_puts("entered rack wala \r\n");
		if((!(RACK_STATUSPORT & (1<<RACK_STATUSPIN))) && (where == inLZ2 ) && !ManualInFrontOfLZ2){
			//'f' is sent to throwing mechanism to tell to throwing zone 3 from golden rack
			task6 = task7 = true;
			task8 = task9 = false;
			/*//uart0_puts("going throwing zone 3\r\n");*/
			uart3_putc('j');
			GoldenRackGiven = true;
			ShuttleCockArmGone = true;
			ShuttleCockGiven = true;
			GoThrowingZone3 = true;
			robotState = moving;
			GoThrowingZone2 = false;
		}
		//if rack is not above geneva but robot is in loading zone 2
		else if(where == inLZ2){
			////uart0_puts("none \r\n");
			GoldenRackGiven = false;
			GoThrowingZone2 = true;
			GoThrowingZone3 = false;
			GoldenRackGiven = false;
		}
	}
	
 	
}


/*When Junction on Linetracker X is detected*/
ISR(PCINT0_vect)		
{
	if(!task1){
		task1 = true;
		//reached to corner of loading zone1
		FlagChangeSetpointCompass = true;
	}
	else if(!task5){
		task5 = true;			//reached loading zone 2 from loading zone 1
		FlagChangeSetpointCompass = true;
		BrakeMotor();
	}
	else if(!xJunctionMeetFromTZ2){
		xJunctionMeetFromTZ2 = true;
		BrakeMotor();
		linetrackerXjunctionWatchOff();
		encoderX.resetCount();
		encoderY.resetCount();
	}
}


/*When Junction on Linetracker Y is detected*/
ISR(PCINT2_vect)
{
	if(!task2){
		task2 = true;		//reached to loading zone 1
		BrakeMotor();
	}
	else if(!task3){
		_b_Transmit_once = true;
		task3 = true;		//reached throwing zone 1
		BrakeMotor();
	}
		
	else if(!task4){
		task4 = true;		//reached loading zone 1 from throwing zone 1
		 BrakeMotor();
	}
	else if(!task6){
		_b_Transmit_once = true;
		task6 = true;		//reached throwing zone 2
		BrakeMotor();
	}
		
	else if(!task7){
		task7 = true;		//reached loading zone 2 from throwing zone 2
		BrakeMotor();
	}
	else if(!task8){
		_b_Transmit_once = true;
		task8 = true;		//reached throwing zone 3
		BrakeMotor();
	}
	else if(!task9){
		task9 = true;		//reached loading zone 2 from throwing zone 3
		BrakeMotor();
	}
}


#endif /* ZONEDRIVE_H_ */