#include "kilolib.h"
//#define DEBUG
//#include "debug.h" // for real robots only
//#include <stdio.h> // for ARGOS only
#include <stdlib.h>
#include <math.h>
#include <float.h>

#define PI 3.14159265358979323846

/* Enum for different motion types */
typedef enum {
    STOP = 0,
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
} motion_t;

/* Enum for boolean flags */
typedef enum {
    false = 0,
    true = 1,
} bool;


/* current motion type */
motion_t current_motion_type = STOP;

/* current LED color */
uint16_t current_LED_color=RGB(0,0,0);


unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 150; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint32_t max_straight_ticks = 300;
uint32_t last_motion_ticks = 0;
uint32_t update_ticks = 60; /* setting how often performing the commitment update. a tick here is every ~31ms */
uint32_t last_update_ticks = 0;

/* Variables for outgoing messages */
message_t message;


/* Variables for incoming messages from ARK */
uint8_t RED_GOAL_GPS_X;
uint8_t RED_GOAL_GPS_Y;
uint8_t GREEN_GOAL_GPS_X;
uint8_t GREEN_GOAL_GPS_Y;


/* Robot GPS variables */
uint8_t Robot_GPS_X;
uint8_t Robot_GPS_Y;
double Robot_orientation;
bool new_sa_msg_gps=false;

/* Robot Goal variables*/
uint8_t Goal_GPS_X;
uint8_t Goal_GPS_Y;

/* Variables to check whether it is inside the areas */
bool insideRedArea;
bool insideGreenArea;

/* Wall Avoidance manouvers */
uint32_t wallAvoidanceCounter=0; // to decide when the robot is stuck...

/* Options lookup table*/


/* RTID variables */
bool runtime_identification=false;
uint32_t backup_kiloticks;
uint16_t backup_LED;
motion_t backup_motion=STOP;


float RotSpeed=38.0;

uint8_t GPS_maxcell=16;
uint8_t minDist=4;

float GPS_To_Meter=1/16.0;
float Robot_FoV=0.0;


bool debug_state=false;
uint32_t debug_lastTableUpdate = 0;
uint8_t debug_lastAssignedID = 0;
uint8_t debug_lastSource = 0;


bool goals_set = false;


/*-------------------------------------------------------------------*/
/* Compute angle to Goal                                             */
/*-------------------------------------------------------------------*/
void NormalizeAngle(double* angle)
{
    while(*angle>180){
        *angle=*angle-360;
    }
    while(*angle<-180){
        *angle=*angle+360;
    }
}


/*-------------------------------------------------------------------*/
/* Compute angle to Goal                                             */
/*-------------------------------------------------------------------*/
double AngleToGoal() {
    NormalizeAngle(&Robot_orientation);
    double angletogoal=atan2(Goal_GPS_Y-Robot_GPS_Y,Goal_GPS_X-Robot_GPS_X)/PI*180-Robot_orientation;
    NormalizeAngle(&angletogoal);
    return angletogoal;
}


/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
    if( current_motion_type != new_motion_type )
    {
        current_motion_type = new_motion_type;

        int calibrated = true;
        switch( new_motion_type )
        {
        case FORWARD:
            if(!runtime_identification)
                spinup_motors();
            if (calibrated){

                if(!runtime_identification)
                    set_motors(kilo_straight_left,kilo_straight_right);
            }
            else
            {
                if(!runtime_identification)
                    set_motors(67,67);
            }
            break;
        case TURN_LEFT:
            if(!runtime_identification)
                spinup_motors();
            if (calibrated)
            {
                if(!runtime_identification) {
                    uint8_t leftStrenght = kilo_turn_left;
                    uint8_t i;
                    for (i=3; i <= 18; i += 3){
                        if (wallAvoidanceCounter >= i){
                            leftStrenght+=2;
                        }
                    }
                    set_motors(leftStrenght,0);
                    //                    set_motors(kilo_turn_left,0);
                }
            }
            else{
                if(!runtime_identification)
                    set_motors(70,0);
            }
            break;
        case TURN_RIGHT:
            if(!runtime_identification)
                spinup_motors();
            if (calibrated){
                if(!runtime_identification) {
                    uint8_t rightStrenght = kilo_turn_right;
                    uint8_t i;
                    for (i=3; i <= 18; i += 3){
                        if (wallAvoidanceCounter >= i){
                            rightStrenght+=2;
                        }
                    }
                    set_motors(0,rightStrenght);
                    //                    set_motors(0,kilo_turn_right);
                }
            }
            else{
                if(!runtime_identification)
                    set_motors(0,70);
            }
            break;
        case STOP:
        default:
            set_motors(0,0);
        }

        if(current_motion_type!=STOP){
            backup_motion=current_motion_type;
        }
    }
}


/*-------------------------------------------------------------------*/
/* Function for setting the new goal                                 */
/* (including LED colour and message initialisation)                 */
/*-------------------------------------------------------------------*/
void set_goal( uint8_t new_goal_GPS_X, uint8_t new_goal_GPS_Y)
{
    Goal_GPS_X = new_goal_GPS_X;
    Goal_GPS_Y = new_goal_GPS_Y;
}


/*** SCT-related code ***/
/*
* This code was originally generated by Nadzoru 2.
*
* To use the generated control logic, implement the functions that have the following names:
*
* - input_read_[event_name]: This function should return 1 if the event [event_name] is detected, and 0 otherwise.
* - callback_[event_name]: This function is called when the event [event_name] is triggered.
*/
#define NUM_EVENTS 8
#define NUM_SUPERVISORS 4

#define EV_work 0
#define EV_notAtCharger 1
#define EV_charge 2
#define EV_atCharger 3
#define EV_atWork 4
#define EV_moveToCharge 5
#define EV_notAtWork 6
#define EV_moveToWork 7


const unsigned char ev_controllable[8] = {1, 0, 1, 0, 0, 1, 0, 1};
const unsigned char sup_events[4][8] = {{1, 1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1, 1}};
const unsigned long int sup_init_state[4] = {1, 2, 0, 1};
unsigned long int sup_current_state[4] = {1, 2, 0, 1};
const unsigned long int sup_data_pos[4] = {0, 125, 250, 375};
const unsigned char sup_data[500] = {5, EV_atCharger, 0, 0, EV_notAtWork, 0, 0, EV_atWork, 0, 2, EV_notAtCharger, 0, 0, EV_work, 0, 3, 4, EV_notAtWork, 0, 6, EV_atCharger, 0, 1, EV_atWork, 0, 1, EV_notAtCharger, 0, 1, 5, EV_atCharger, 0, 2, EV_notAtCharger, 0, 2, EV_notAtWork, 0, 0, EV_work, 0, 5, EV_atWork, 0, 2, 5, EV_atCharger, 0, 3, EV_notAtWork, 0, 3, EV_atWork, 0, 5, EV_moveToCharge, 0, 7, EV_notAtCharger, 0, 3, 5, EV_notAtCharger, 0, 4, EV_atWork, 0, 4, EV_charge, 0, 1, EV_notAtWork, 0, 7, EV_atCharger, 0, 4, 5, EV_atCharger, 0, 5, EV_moveToCharge, 0, 4, EV_atWork, 0, 5, EV_notAtCharger, 0, 5, EV_notAtWork, 0, 3, 5, EV_moveToWork, 0, 0, EV_notAtWork, 0, 6, EV_notAtCharger, 0, 6, EV_atCharger, 0, 6, EV_atWork, 0, 1, 5, EV_notAtCharger, 0, 7, EV_atWork, 0, 4, EV_atCharger, 0, 7, EV_notAtWork, 0, 7, EV_charge, 0, 6, 5, EV_atWork, 0, 0, EV_work, 0, 5, EV_atCharger, 0, 1, EV_notAtWork, 0, 0, EV_notAtCharger, 0, 0, 5, EV_atWork, 0, 1, EV_work, 0, 7, EV_notAtWork, 0, 1, EV_atCharger, 0, 1, EV_notAtCharger, 0, 0, 5, EV_atWork, 0, 2, EV_moveToWork, 0, 1, EV_notAtWork, 0, 2, EV_atCharger, 0, 2, EV_notAtCharger, 0, 6, 5, EV_atCharger, 0, 4, EV_notAtCharger, 0, 3, EV_atWork, 0, 3, EV_notAtWork, 0, 3, EV_charge, 0, 6, 5, EV_notAtCharger, 0, 3, EV_atWork, 0, 4, EV_notAtWork, 0, 4, EV_atCharger, 0, 4, EV_charge, 0, 2, 5, EV_moveToCharge, 0, 3, EV_notAtCharger, 0, 5, EV_atCharger, 0, 7, EV_atWork, 0, 5, EV_notAtWork, 0, 5, 5, EV_moveToWork, 0, 0, EV_atWork, 0, 6, EV_notAtWork, 0, 6, EV_notAtCharger, 0, 6, EV_atCharger, 0, 2, 4, EV_atCharger, 0, 7, EV_atWork, 0, 7, EV_notAtCharger, 0, 5, EV_notAtWork, 0, 7, 5, EV_atWork, 0, 2, EV_moveToWork, 0, 1, EV_atCharger, 0, 0, EV_notAtCharger, 0, 0, EV_notAtWork, 0, 0, 4, EV_notAtWork, 0, 1, EV_atCharger, 0, 1, EV_notAtCharger, 0, 1, EV_atWork, 0, 3, 5, EV_notAtWork, 0, 0, EV_notAtCharger, 0, 2, EV_atCharger, 0, 2, EV_atWork, 0, 2, EV_moveToWork, 0, 3, 5, EV_work, 0, 5, EV_atCharger, 0, 3, EV_notAtCharger, 0, 3, EV_notAtWork, 0, 1, EV_atWork, 0, 3, 5, EV_charge, 0, 0, EV_atCharger, 0, 4, EV_notAtCharger, 0, 4, EV_atWork, 0, 7, EV_notAtWork, 0, 4, 5, EV_notAtCharger, 0, 5, EV_atCharger, 0, 5, EV_atWork, 0, 5, EV_moveToCharge, 0, 7, EV_notAtWork, 0, 6, 5, EV_atCharger, 0, 6, EV_notAtWork, 0, 6, EV_notAtCharger, 0, 6, EV_moveToCharge, 0, 4, EV_atWork, 0, 5, 5, EV_atCharger, 0, 7, EV_charge, 0, 2, EV_atWork, 0, 7, EV_notAtWork, 0, 4, EV_notAtCharger, 0, 7, 5, EV_notAtWork, 0, 0, EV_moveToWork, 0, 4, EV_notAtCharger, 0, 1, EV_atWork, 0, 0, EV_atCharger, 0, 0, 5, EV_moveToWork, 0, 2, EV_atCharger, 0, 0, EV_atWork, 0, 1, EV_notAtWork, 0, 1, EV_notAtCharger, 0, 1, 5, EV_atCharger, 0, 4, EV_atWork, 0, 2, EV_notAtWork, 0, 2, EV_work, 0, 5, EV_notAtCharger, 0, 2, 4, EV_notAtCharger, 0, 3, EV_notAtWork, 0, 3, EV_atWork, 0, 3, EV_atCharger, 0, 7, 5, EV_notAtWork, 0, 4, EV_atWork, 0, 4, EV_notAtCharger, 0, 2, EV_work, 0, 6, EV_atCharger, 0, 4, 5, EV_atCharger, 0, 6, EV_notAtWork, 0, 5, EV_notAtCharger, 0, 5, EV_moveToCharge, 0, 3, EV_atWork, 0, 5, 5, EV_atCharger, 0, 6, EV_moveToCharge, 0, 7, EV_notAtWork, 0, 6, EV_atWork, 0, 6, EV_notAtCharger, 0, 5, 5, EV_charge, 0, 0, EV_notAtCharger, 0, 3, EV_atWork, 0, 7, EV_notAtWork, 0, 7, EV_atCharger, 0, 7};


unsigned long int get_state_position(unsigned char supervisor, unsigned long int state){
    unsigned long int position;
    unsigned long int s;
    unsigned char en;
    position = sup_data_pos[supervisor];
    for(s=0; s<state; s++){
        en = sup_data[position];
        position += en*3 + 1;
    }
    return position;
}

void make_transition(unsigned char event){
	unsigned char i;
	unsigned long int position;
	unsigned char num_transitions;

    for(i=0; i<NUM_SUPERVISORS; i++){
        if(sup_events[i][event]){
            position        = get_state_position(i, sup_current_state[i]);
            num_transitions = sup_data[position];
            position++;
            while(num_transitions--){
                if(sup_data[position] == event){
                    sup_current_state[i] = ((unsigned long int)sup_data[position + 1])*256 + ((unsigned long int)sup_data[position + 2]);
                    break;
                }
                position += 3;
            }
        }
    }
}

/* IN_read */
unsigned char input_buffer[256];
unsigned char input_buffer_pnt_add = 0;
unsigned char input_buffer_pnt_get = 0;

unsigned char input_buffer_get(unsigned char *event){
    if(input_buffer_pnt_add == input_buffer_pnt_get){
        return 0;
    } else {
        *event = input_buffer[ input_buffer_pnt_get ];
        input_buffer_pnt_get++;
        return 1;
    }
}

void input_buffer_add(unsigned char event){
    input_buffer[input_buffer_pnt_add] = event;
    input_buffer_pnt_add++;
}

unsigned char input_buffer_check_empty(  ){
    return input_buffer_pnt_add == input_buffer_pnt_get;
}


unsigned char input_read_notAtCharger(){
    // type your code for inputs
    return 0;
}
    
unsigned char input_read_atCharger(){
    // type your code for inputs
    return 0;
}
    
unsigned char input_read_atWork(){
    // type your code for inputs
    return 0;
}
    
unsigned char input_read_notAtWork(){
    // type your code for inputs
    return 0;
}
    

unsigned char input_read(unsigned char ev){
    unsigned char result = 0;
    switch(ev){
        case EV_notAtCharger:
            result = input_read_notAtCharger();
            break;
        case EV_atCharger:
            result = input_read_atCharger();
            break;
        case EV_atWork:
            result = input_read_atWork();
            break;
        case EV_notAtWork:
            result = input_read_notAtWork();
            break;}
    return result;
}
void callback_work(){
    // type your code here
}

void callback_notAtCharger(){
    return !insideGreenArea;
}

void callback_charge(){
    // type your code here
}

void callback_atCharger(){
    return insideGreenArea;
}

void callback_atWork(){
    return insideRedArea;
}

void callback_moveToCharge(){
    set_goal(GREEN_GOAL_GPS_X,GREEN_GOAL_GPS_Y);
    set_color(RGB(0,3,0));
}

void callback_notAtWork(){
    return !insideRedArea;
}

void callback_moveToWork(){
    set_goal(RED_GOAL_GPS_X,RED_GOAL_GPS_Y);
    set_color(RGB(3,0,0));
}


void callback(char ev){
    switch(ev){
        case EV_work:
            callback_work();
            break;
        case EV_notAtCharger:
            callback_notAtCharger();
            break;
        case EV_charge:
            callback_charge();
            break;
        case EV_atCharger:
            callback_atCharger();
            break;
        case EV_atWork:
            callback_atWork();
            break;
        case EV_moveToCharge:
            callback_moveToCharge();
            break;
        case EV_notAtWork:
            callback_notAtWork();
            break;
        case EV_moveToWork:
            callback_moveToWork();
            break;
        }
}

void get_active_controllable_events(unsigned char *events){
    unsigned char i, j;

    /* Disable all non controllable events */
    for (i = 0; i < NUM_EVENTS; i++)
    {
        if (!ev_controllable[i])
        {
            events[i] = 0;
        }
    }

    /* Check disabled events for all supervisors */
    for (i = 0; i < NUM_SUPERVISORS; i++){
        unsigned long int position;
        unsigned char ev_disable[NUM_EVENTS], k;
        unsigned char num_transitions;
        for (k = 0; k < NUM_EVENTS; k++){
            ev_disable[k] = 1;
        }
        for (j = 0; j < NUM_EVENTS; j++){

            /*if supervisor don't have this event, it can't disable the event*/
            if (!sup_events[i][j]){
                ev_disable[j] = 0;
            }
        }
        /*if supervisor have a transition with the event in the current state, it can't disable the event */
        position = get_state_position(i, sup_current_state[i]);
        num_transitions = sup_data[position];
        position++;
        while (num_transitions--){
            ev_disable[sup_data[position]] = 0;
            position += ((unsigned long int)3);
        }

        /* Disable for current supervisor states */
        for(j=0; j<NUM_EVENTS; j++){
            if(ev_disable[j] == 1){
                events[j] = 0;
            }
        }
    }
}

/*choices*/
unsigned char last_events[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char get_next_controllable(unsigned char *event){
    unsigned char events[8] = {1, 1, 1, 1, 1, 1, 1, 1};

    int count_actives, random_pos;
    unsigned char i;

    get_active_controllable_events(events);
    count_actives = 0;
    for (i = 0; i < NUM_EVENTS; i++){
        if (events[i]){
            count_actives++;
        }
    }
    if (count_actives){
        random_pos = rand_soft() % count_actives;
        for (i = 0; i < NUM_EVENTS; i++){
            if (!random_pos && events[i]){
                *event = i;
                return 1;
            } else if (events[i]){
                random_pos--;
            }
        }
    }
    return 0;
}

void update_input(){
    unsigned char i;
    for(i=0;i<NUM_EVENTS;i++){
        if( !ev_controllable[i]){
            if(  input_read( i ) ){
                if( !last_events[i] ){
                    input_buffer_add( i );
                    last_events[i] = 1;
                }
            } else {
                last_events[i] = 0;
            }
        }
    }
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
    /* Initialise commitment and LED */
    //    set_commitment(0 , 0 , 0);

    /* Initialise motors */
    set_motors(0,0);
    kilo_ticks=0;

    /* Initialise random seed */
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    //    /* Initialise motion variables */
    set_motion( STOP );
    last_motion_ticks = rand() % max_straight_ticks + 1;

    //    /* Initialise broadcast variables */
    //    last_broadcast_ticks = rand() % broadcast_ticks + 1;

    //    /** Initialise update variables */
    //    last_update_ticks= rand() % update_ticks;


    //    /* Intialize time to 0 */
    //    kilo_ticks=0;

    /* initialise the GSP to the middle of the environment, to avoid to trigger wall avoidance immediately */
    Robot_GPS_X = GPS_maxcell/2;
    Robot_GPS_Y = GPS_maxcell/2;
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx( message_t *msg, distance_measurement_t *d ) {
    /** ARK MESSAGE **/
    if (msg->type == 0) {

        // unpack message
        int id1 = msg->data[0];
        int id2 = msg->data[3];
        int id3 = msg->data[6];
        unsigned int sa_type;

        if (id1 == kilo_uid) {
            // unpack type
            sa_type = msg->data[1] >> 7;
            if(sa_type==0){
                // unpack payload
                Robot_GPS_X = msg->data[1]>>2 & 0x1F;
                Robot_GPS_Y = (msg->data[1] & 0x03)<< 3 | msg->data[2]>>5 ;
                Robot_orientation = (msg->data[2] & 0x1F)*12;

//                if (Robot_GPS_Y == 26)
//                    set_color(RGB(3,3,3));

                //                printf("My GPS coords are: ( %d , %d ) \n", Robot_GPS_X , Robot_GPS_Y);
                //                printf("My orientation is: %f\n", Robot_orientation);
                new_sa_msg_gps = true;
            }

        }
        if (id2 == kilo_uid) {
            sa_type = msg->data[4] >> 7;
            if(sa_type==0){
                // unpack payload
                Robot_GPS_X = msg->data[4]>>2 & 0x1F;
                Robot_GPS_Y = (msg->data[4] & 0x03)<< 3 | msg->data[5]>>5 ;
                Robot_orientation = (msg->data[5] & 0x1F)*12;
                new_sa_msg_gps = true;

//                if (Robot_GPS_Y == 26)
//                    set_color(RGB(3,3,3));

            }
        }
        if (id3 == kilo_uid) {
            sa_type = msg->data[7] >> 7;
            if(sa_type==0){
                // unpack payload
                Robot_GPS_X = msg->data[7]>>2 & 0x1F;
                Robot_GPS_Y = (msg->data[7] & 0x03)<< 3 | msg->data[8]>>5 ;
                Robot_orientation = (msg->data[8] & 0x1F)*12;
                new_sa_msg_gps = true;

//                if (Robot_GPS_Y == 26)
//                    set_color(RGB(3,3,3));
            }
        }

    }


    /** ARK Config File **/
    else if (msg->type == 10) {

        if (msg->data[0] == kilo_uid)
        {
            //            set_color(RGB(3,3,3));
            GREEN_GOAL_GPS_X = msg->data[1];
            GREEN_GOAL_GPS_Y = msg->data[2];

            RED_GOAL_GPS_X = msg->data[3];
            RED_GOAL_GPS_Y = msg->data[4];

            //            if (RED_GOAL_GPS_Y == 26)
            //                set_color(RGB(3,3,3));

            set_goal(RED_GOAL_GPS_X,RED_GOAL_GPS_Y);

            //            if (Goal_GPS_Y == Goal_GPS_Y)
            //            {
            set_color(RGB(3,0,0));
            //            }

            goals_set = true;
            set_motion( FORWARD );
        }
    }
    /** ARK ID identification **/
    else if (msg->type == 120) {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }
    }
    /** ARK Runtime identification **/
    else if (msg->type == 119) {
        // runtime identification
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id >= 0){ // runtime identification ongoing
            set_motion(STOP);
            runtime_identification = true;
            if (id == kilo_uid) {
                set_color(RGB(0,0,3));
            } else {
                set_color(RGB(3,0,0));
            }
        } else { // runtime identification ended
            kilo_ticks=backup_kiloticks;
            //set_color(current_LED_color);
            runtime_identification = false;
            set_motion(backup_motion);
        }
    }

}

void CheckAreasSensor(){
    if(new_sa_msg_gps)
    {
        //        new_sa_msg_gps=false;
        if (Goal_GPS_Y == RED_GOAL_GPS_Y && Robot_GPS_Y >= RED_GOAL_GPS_Y)
        {
            insideRedArea = true;
        }
        else
        {
            insideRedArea = false;
        }

        if (Goal_GPS_Y == GREEN_GOAL_GPS_Y && Robot_GPS_Y <= GREEN_GOAL_GPS_Y)
        {
            insideGreenArea = true;
        }
        else
        {
            insideGreenArea = false;
        }
    }
}
/*-------------------------------------------------------------------*/
/* Function to go to the Goal location (e.g. to resample an option)  */
/*-------------------------------------------------------------------*/
void GoToGoalLocation(){
    if(new_sa_msg_gps){
        new_sa_msg_gps=false;

        double angleToGoal = AngleToGoal();
        if(fabs(angleToGoal) <= 20)
        {
            set_motion(FORWARD);
            last_motion_ticks = kilo_ticks;
        }
        else{
            if(angleToGoal>0){
                set_motion(TURN_LEFT);
                last_motion_ticks = kilo_ticks;
                turning_ticks=(unsigned int) ( fabs(angleToGoal)/RotSpeed*32.0 );
                //                    debug_print("In need to turn left for: %d\n", turning_ticks );
            }
            else{
                set_motion(TURN_RIGHT);
                last_motion_ticks = kilo_ticks;
                turning_ticks=(unsigned int) ( fabs(angleToGoal)/RotSpeed*32.0 );
                //                    debug_print("In need to turn right for: %d\n", turning_ticks );
            }
        }
    }

    switch( current_motion_type ) {    //    debug_init();

    case TURN_LEFT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            //	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
            set_motion(FORWARD);
        }
        break;
    case TURN_RIGHT:
        if( kilo_ticks > last_motion_ticks + turning_ticks ) {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;  // fixed time FORWARD
            //	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
            set_motion(FORWARD);
        }
        break;
    case FORWARD:
        break;

    case STOP:
    default:
        set_motion(STOP);
    }

}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop()
{
    
    if (goals_set)
    {
        CheckAreasSensor();

        // AUTOMATA PLAYER
        update_input();
        unsigned char event;
        while( input_buffer_get( &event ) ){//clear buffer, executing all no controllable events (NCE)
            make_transition( event );
            callback( event );
        }
        if( get_next_controllable( &event ) ){//find controllable event (CE)
            if( input_buffer_check_empty() ){ //Only execute CE if NCE input buffer is empty
                make_transition( event );
                callback( event );
            }
        }

        GoToGoalLocation();
    }
}

/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main(void)
{

    kilo_init();
    kilo_message_rx=message_rx;
    kilo_start(setup, loop);

    return 0;
}
