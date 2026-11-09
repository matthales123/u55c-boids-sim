///////////////////////////////////////
/// 640x480 version! 16-bit color
/// This code will segfault the original
/// DE1 computer
/// compile with
/// gcc multiple_boids_VGA.c -o mult_boids -O3 -lm
///
///////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h> 
#include <math.h>
//#include "address_map_arm_brl4.h"

// video display
#define SDRAM_BASE            0xC0000000
#define SDRAM_END             0xC3FFFFFF
#define SDRAM_SPAN			  0x04000000
// characters
#define FPGA_CHAR_BASE        0xC9000000 
#define FPGA_CHAR_END         0xC9001FFF
#define FPGA_CHAR_SPAN        0x00002000
/* Cyclone V FPGA devices */
#define HW_REGS_BASE          0xff200000
//#define HW_REGS_SPAN        0x00200000 
#define HW_REGS_SPAN          0x00005000 

// graphics primitives
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_rect (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;
void VGA_Vline(int, int, int, short) ;
void VGA_Hline(int, int, int, short) ;
void VGA_disc (int, int, int, short);
void VGA_circle (int, int, int, int);

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// uS per frame
#define FRAME_RATE 1000000/30 //33000
#define PI 3.1415927
// #define BLACK 0x00
// #define WHITE 0xFF

// 16-bit primary colors
#define red  (0+(0<<5)+(31<<11))
#define dark_red (0+(0<<5)+(15<<11))
#define green (0+(63<<5)+(0<<11))
#define dark_green (0+(31<<5)+(0<<11))
#define blue (31+(0<<5)+(0<<11))
#define dark_blue (15+(0<<5)+(0<<11))
#define yellow (0+(63<<5)+(31<<11))
#define cyan (31+(63<<5)+(0<<11))
#define magenta (31+(0<<5)+(31<<11))
#define black (0x0000)
#define gray (15+(31<<5)+(51<<11))
#define white (0xffff)
int colors[] = {red, dark_red, green, dark_green, blue, dark_blue, 
		yellow, cyan, magenta, gray, black, white};

// pixel macro
#define VGA_PIXEL(x,y,color) do{\
	int  *pixel_ptr ;\
	pixel_ptr = (int*)((char *)vga_pixel_ptr + (((y)*640+(x))<<1)) ; \
	*(short *)pixel_ptr = (color);\
} while(0)

// the light weight buss base
void *h2p_lw_virtual_base;

// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

// /dev/mem file id
int fd;

//? Define the turn factor for adjusting the boid's direction when near an edge
fix15 turnfactor = float2fix15(0.2);
//? Define the visual range for boids to perceive other boids within this distance
fix15 visualRange = float2fix15(40);
//? Define the protected range for boids to maintain a safe distance from other boids
fix15 protectedRange = float2fix15(8);
//? Define the centering factor for how strongly boids try to match the position of other boids
fix15 centeringfactor = float2fix15(0.0005);
//? Define the avoidance factor for how strongly boids try to avoid other boids within the protected range
fix15 avoidfactor = float2fix15(0.05);
//? Define the matching factor for how strongly boids try to match the velocity of other boids
fix15 matchingfactor = float2fix15(0.05);
//? Define the maximum speed a boid can move
fix15 maxspeed = float2fix15(6);
//? Define the minimum speed a boid can move
fix15 minspeed = float2fix15(3);
//? Define the maximum bias value to change boid's direction
fix15 maxbias = float2fix15(0.01);
//? Define the increment value for bias adjustment
fix15 bias_increment = float2fix15(0.00004);
//? Define the initial bias value for adjusting the boid's direction
fix15 biasval = float2fix15(0.001);


//*========================================================================
//* Boid Initialization and Spawning
//*========================================================================
typedef struct {
    //? x-coordinate of the boid's position in fixed-point format
    fix15 x;
    //? y-coordinate of the boid's position in fixed-point format
    fix15 y;
    //? x-component of the boid's velocity in fixed-point format
    fix15 vx;    
    //? y-component of the boid's velocity in fixed-point format
    fix15 vy; 
    //? color of the boid represented by a character value   
    char color; 
} Boid;



//!========================================================================
//! NUMBER OF BOIDS
//!========================================================================
#define NUM_BOIDS 2
Boid boids[NUM_BOIDS];
//!========================================================================


//*========================================================================
//* Creating Multiple Boids
//*========================================================================
void spawnBoid(Boid* boid) {
    //? Give Random x and y coordinates to each boid
    boid->x = int2fix15(rand() % (540-100) + 100);
    boid->y = int2fix15(rand() % (380-100) + 100);


    //? Give Random Speed and Angle to each boid
    float random_speed = (float)rand() / (float)(RAND_MAX) * (6-3) + 3;
    float random_angle = (float)rand() / (float)(RAND_MAX) * (2*PI);

    //? Assign boids x and y velocity from the random speed and angle
    boid->vx = float2fix15(random_speed * (float) cos((double) random_angle));
    boid->vy = float2fix15(random_speed * (float) sin((double) random_angle));
    boid->color = WHITE; // Set the default color to WHITE
}

//*========================================================================
//* Updating Each Boid
//*========================================================================
void boid_update(int cur) {
    //? Get a pointer to the current boid
    Boid *boid = &boids[cur];
    
    //? Initialize variables for calculating the average position and velocity of neighboring boids, and the distance to close boids
    fix15 xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, close_dx = 0, close_dy = 0;
    //? Initialize a counter for the number of neighboring boids
    fix15 neighboring_boids = 0;
    //? Loop through all the boids
    for (int i = 0; i < NUM_BOIDS; i++) {
        //? Skip the current boid
        if (i != cur) {
            //? Get a pointer to the other boid
            Boid *other_boid = &boids[i];

            //? Calculate the difference in x and y coordinates between the current boid and the other boid, accounting for wrapping
            fix15 dx = boid->x - other_boid->x;
            if (dx > int2fix15(320)) {dx = dx - int2fix15(640);}
            if (dx < int2fix15(-320)) {dx = dx + int2fix15(640);}
            fix15 dy = boid->y - other_boid->y;
            if (dy > int2fix15(240)) {dy = dy - int2fix15(480);}
            if (dy < int2fix15(-240)) {dy = dy + int2fix15(480);}

            //? Check if the other boid is within the current boid's visual range
            if (dx < visualRange && dy < visualRange && dx > -visualRange && dy > -visualRange) {
                //? Calculate the squared distance between the current boid and the other boid
                fix15 squared_distance = multfix15(dx, dx) + multfix15(dy, dy);

                //? Check if the other boid is within the current boid's protected range
                if (squared_distance < multfix15(protectedRange, protectedRange)) {
                    //? If so, accumulate the distance to the close boid
                    close_dx += dx;
                    close_dy += dy;
                //? Check if the other boid is within the current boid's visual range but outside the protected range
                } else if (squared_distance < multfix15(visualRange, visualRange)) {
                    //? If so, accumulate the position and velocity of the neighboring boid
                    xpos_avg += other_boid->x;
                    ypos_avg += other_boid->y;
                    xvel_avg += other_boid->vx;
                    yvel_avg += other_boid->vy;

                    //? Increment the counter for neighboring boids
                    neighboring_boids += int2fix15(1);
                }
            }
        }
    }

    //? If there are neighboring boids within the visual range
    if (neighboring_boids > 0) {
        //? Calculate the average position and velocity of neighboring boids
        xpos_avg = divfix(xpos_avg, neighboring_boids);
        ypos_avg = divfix(ypos_avg, neighboring_boids);
        xvel_avg = divfix(xvel_avg, neighboring_boids);
        yvel_avg = divfix(yvel_avg, neighboring_boids);

        //? Adjust the boid's velocity based on average position and velocity of neighboring boids
        boid->vx = boid->vx + multfix15((xpos_avg - boid->x), centeringfactor) + multfix15((xvel_avg - boid->vx), matchingfactor);
        boid->vy = boid->vy + multfix15((ypos_avg - boid->y), centeringfactor) + multfix15((yvel_avg - boid->vy), matchingfactor);
    }

    //? Adjust the boid's velocity to avoid close boids
    boid->vx = boid->vx + multfix15(close_dx, avoidfactor);
    boid->vy = boid->vy + multfix15(close_dy, avoidfactor);

    //? Check and adjust the boid's position based on the boundary conditions
    boundary_check(cur);

    //? Apply a bias to the boid's velocity based on its index
    if (cur >= 0 && cur < 25) {
        boid->vx = multfix15((int2fix15(1) - biasval), (boid->vx)) + biasval;
    } else if (cur >= 25 && cur < 50) {
        boid->vx = multfix15((int2fix15(1) - biasval), (boid->vx)) - biasval;
    }

    //? Calculate the boid's current speed
    fix15 temp_add = multfix15(boid->vx, boid->vx) + multfix15(boid->vy, boid->vy);
    fix15 speed = sqrtfix(temp_add);

    //? Ensure the boid's speed is within the minspeed and maxspeed limits
    if (speed < minspeed) {
        boid->vx = multfix15(divfix(boid->vx, speed), minspeed);
        boid->vy = multfix15(divfix(boid->vy, speed), minspeed);
    }
    if (speed > maxspeed) {
        boid->vx = multfix15(divfix(boid->vx, speed), maxspeed);
        boid->vy = multfix15(divfix(boid->vy, speed), maxspeed);
    }

    //? Update the boid's position based on its velocity
    boid->x = boid->x + boid->vx;
    //? Wrap the boid's position around the horizontal edges of the screen
    if (boid->x > int2fix15(640)) {
        boid->x = boid->x - int2fix15(640);
    }
    if (boid->x < int2fix15(0)) {
        boid->x = boid->x + int2fix15(640);
    }
    boid->y = boid->y + boid->vy;
    //? Wrap the boid's position around the vertical edges of the screen
    if (boid->y > int2fix15(480)) {
        boid->y = boid->y - int2fix15(480);
    }
    if (boid->y < int2fix15(0)) {
        boid->y = boid->y + int2fix15(480);
    }
}


//*========================================================================
//* Updating Boid velocities per interaction with Walls and Edges
//*========================================================================
void wallsAndEdges(Boid *boid) {
    //? Check if the boid has hit the top boundary of the screen, 
    //? and reverse its vertical velocity if it has
    if (hitTop(boid->y)) {
        boid->vy = boid->vy + turnfactor;
    }
    //? Check if the boid has hit the bottom boundary of the screen, 
    //? and reverse its vertical velocity if it has
    if (hitBottom(boid->y)) {
        boid->vy = boid->vy - turnfactor;
    }
    //? Check if the boid has hit the right boundary of the screen, 
    //? and reverse its horizontal velocity if it has
    if (hitRight(boid->x)) {
        boid->vx = boid->vx - turnfactor;
    }
    //? Check if the boid has hit the left boundary of the screen, 
    //? and reverse its horizontal velocity if it has
    if (hitLeft(boid->x)) {
        boid->vx = boid->vx + turnfactor;
    }

    //? Update the boid's position based on its current velocity
    boid->x = boid->x + boid->vx;
    boid->y = boid->y + boid->vy;
}


//*========================================================================
//* Updating Boid velocities per interaction with boundaries
//*========================================================================
void boundary_check(Boid *boid) {
    //? Calculate the half value of the bound_type variable
    fix15 half = bound_type >> 1;
    //? Calculate the left boundary value
    fix15 left = int2fix15(320) - half;
    //? Calculate the right boundary value
    fix15 right = int2fix15(320) + half;

    //? Check if the bound_type is 0 (rectangle)
    if (bound_type == 0) {
        //? If the boid is near the top edge, make it turn by turnfactor
        if (hitTop(boid->y)) {
            boid->vy = boid->vy + turnfactor;
        }
        //? If the boid is near the bottom edge, make it turn by turnfactor
        if (hitBottom(boid->y)) {
            boid->vy = boid->vy - turnfactor;
        }
        //? If the boid is near the right edge, make it turn by turnfactor
        if (hitRight(boid->x)) {
            boid->vx = boid->vx - turnfactor;
        }
        //? If the boid is near the left edge, make it turn by turnfactor
        if (hitLeft(boid->x)) {
            boid->vx = boid->vx + turnfactor;
        }

    //? If bound_type is greater than 0 (top/bot, left/right)
    } else if (bound_type > 0) {
        //? If the boid is past the right boundary, make it turn by turnfactor
        if (boid->x > right) {
            boid->vx = boid->vx - turnfactor;
        }
        //? If the boid is past the left boundary, make it turn by turnfactor
        if (boid->x < left) {
            boid->vx = boid->vx + turnfactor;
        }
    }
}



//*========================================================================
//* Arena Creation and Boundaries
//*========================================================================
//? Must be redrawn every time to ensure that Arena does not get overwritten
//? with black spots
void drawArena() {
    VGA_line(100,100,100,380,(short)white);
    VGA_line(540,100,540,380,(short)white);
    VGA_line(100,100,540,100,(short)white);
    VGA_line(100,380,540,380,(short)white);
}


//*========================================================================
//* Measure Time for EACH iteration of the MAIN function
//*========================================================================
struct timeval t1, t2;
double elapsedTime;

//!========================================================================
//! Main
//!========================================================================
int main(void)
{
  	
	// === need to mmap: =======================
	// FPGA_CHAR_BASE
	// FPGA_ONCHIP_BASE      
	// HW_REGS_BASE        
  
	// === get FPGA addresses ==================
    // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
    
    // get virtual addr that maps to physical
	h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
	if( h2p_lw_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap1() failed...\n" );
		close( fd );
		return(1);
	}
    

	// === get VGA char addr =====================
	// get virtual addr that maps to physical
	vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );	
	if( vga_char_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap2() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA LED control 
	vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

	// === get VGA pixel addr ====================
	// get virtual addr that maps to physical
	vga_pixel_virtual_base = mmap( NULL, SDRAM_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_BASE);	
	if( vga_pixel_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA pixel buffer
	vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);

	// ===========================================

	/* create a message to be displayed on the VGA 
          and LCD displays */
	char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
	char text_bottom_row[40] = "Cornell ece5760\0";
	char text_next[40] = "Graphics primitives\0";
	char num_string[20], time_string[20] ;
	char color_index = 0 ;
	int color_counter = 0 ;
	
	// position of disk primitive
	int disc_x = 0;
	// position of circle primitive
	int circle_x = 0 ;
	// position of box primitive
	int box_x = 5 ;
	// position of vertical line primitive
	int Vline_x = 350;
	// position of horizontal line primitive
	int Hline_y = 250;

	//VGA_text (34, 1, text_top_row);
	//VGA_text (34, 2, text_bottom_row);
	// clear the screen
	VGA_box (0, 0, 639, 479, 0x0000);
	// clear the text
	VGA_text_clear();
	// write text
	VGA_text (10, 1, text_top_row);
	VGA_text (10, 2, text_bottom_row);
	VGA_text (10, 3, text_next);
	
	// R bits 11-15 mask 0xf800
	// G bits 5-10  mask 0x07e0
	// B bits 0-4   mask 0x001f
	// so color = B+(G<<5)+(R<<11);


    //? Spawning multiple boids
    for (int i = 0; i < NUM_BOIDS; i++) {
        spawnBoid(&boids[i]);
    }

	while(1) 
	{

		drawArena();

        //? start the timer
        gettimeofday(&t1, NULL);
        
        for (int i = 0; i < NUM_BOIDS; i++) {
            int x = fix2int15(boids[i].x);
            int y = fix2int15(boids[i].y);
            
            //? Erase the previous boid position by drawing a black box
            VGA_box(x, y, x + 2, y + 2, black);
            

            // TODO: Use either of these functions depending on the behavior we want
            // TODO: the boids to follow
            //? Update boid position and velocity using the wallsAndEdges function
            wallsAndEdges(&boids[i]);
            

            //? Update boid's position, velocity, and behavior using the boid_update function
            boid_update(&boids[i]);

            //? Get the updated position of the boid
            x = fix2int15(boids[i].x);
            y = fix2int15(boids[i].y);

            //? Draw the updated boid position using a different color (e.g., white)
            VGA_box(x, y, x + 2, y + 2, boids[i]->color);
        }


        //? Stop timer
        gettimeofday(&t2, NULL);
        //? Calculate elapsed time in microseconds
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000000.0;      // sec to us
        elapsedTime += (t2.tv_usec - t1.tv_usec);   // us 

        //? Convert elapsed time to a string for display
        sprintf(time_string, "T = %6.0f uSec  ", elapsedTime);
        
        //? Display elapsed time on the VGA screen at position (10, 4)
        VGA_text (10, 4, time_string);

        //? Set frame rate by waiting for a fixed time (17 ms) before next iteration
        usleep(17000);

		
	} // end while(1)
} // end main

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset;
	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		// write to the character buffer
		*(character_buffer + offset) = *(text_ptr);	
		++text_ptr;
		++offset;
	}
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor 
****************************************************************************************/
void VGA_text_clear()
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset, x, y;
	for (x=0; x<79; x++){
		for (y=0; y<59; y++){
	/* assume that the text string fits on one line */
			offset = (y << 7) + x;
			// write to the character buffer
			*(character_buffer + offset) = ' ';		
		}
	}
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	for (row = y1; row <= y2; row++)
		for (col = x1; col <= x2; ++col)
		{
			//640x480
			//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
			// set pixel color
			//*(char *)pixel_ptr = pixel_color;	
			VGA_PIXEL(col,row,pixel_color);	
		}
}

/****************************************************************************************
 * Draw a outline rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_rect(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	// left edge
	col = x1;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
		
	// right edge
	col = x2;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
	
	// top edge
	row = y1;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);
	}
	
	// bottom edge
	row = y2;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);
	}
}

/****************************************************************************************
 * Draw a horixontal line on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_Hline(int x1, int y1, int x2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (x1>x2) SWAP(x1,x2);
	// line
	row = y1;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
}

/****************************************************************************************
 * Draw a vertical line on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_Vline(int x1, int y1, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (y2<0) y2 = 0;
	if (y1>y2) SWAP(y1,y2);
	// line
	col = x1;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);			
	}
}


/****************************************************************************************
 * Draw a filled circle on the VGA monitor 
****************************************************************************************/

void VGA_disc(int x, int y, int r, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++)
		for (xc = -r; xc <= r; xc++)
		{
			col = xc;
			row = yc;
			// add the r to make the edge smoother
			if(col*col+row*row <= rsqr+r){
				col += x; // add the center point
				row += y; // add the center point
				//check for valid 640x480
				if (col>639) col = 639;
				if (row>479) row = 479;
				if (col<0) col = 0;
				if (row<0) row = 0;
				//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
				// set pixel color
				//*(char *)pixel_ptr = pixel_color;
				VGA_PIXEL(col,row,pixel_color);	
			}
					
		}
}

/****************************************************************************************
 * Draw a  circle on the VGA monitor 
****************************************************************************************/

void VGA_circle(int x, int y, int r, int pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	int col1, row1;
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++){
		//row = yc;
		col1 = (int)sqrt((float)(rsqr + r - yc*yc));
		// right edge
		col = col1 + x; // add the center point
		row = yc + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
		// left edge
		col = -col1 + x; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
	}
	for (xc = -r; xc <= r; xc++){
		//row = yc;
		row1 = (int)sqrt((float)(rsqr + r - xc*xc));
		// right edge
		col = xc + x; // add the center point
		row = row1 + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
		// left edge
		row = -row1 + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
	}
}

// =============================================
// === Draw a line
// =============================================
//plot a line 
//at x1,y1 to x2,y2 with color 
//Code is from David Rodgers,
//"Procedural Elements of Computer Graphics",1985
void VGA_line(int x1, int y1, int x2, int y2, short c) {
	int e;
	signed int dx,dy,j, temp;
	signed int s1,s2, xchange;
     signed int x,y;
	char *pixel_ptr ;
	
	/* check and fix line coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
        
	x = x1;
	y = y1;
	
	//take absolute value
	if (x2 < x1) {
		dx = x1 - x2;
		s1 = -1;
	}

	else if (x2 == x1) {
		dx = 0;
		s1 = 0;
	}

	else {
		dx = x2 - x1;
		s1 = 1;
	}

	if (y2 < y1) {
		dy = y1 - y2;
		s2 = -1;
	}

	else if (y2 == y1) {
		dy = 0;
		s2 = 0;
	}

	else {
		dy = y2 - y1;
		s2 = 1;
	}

	xchange = 0;   

	if (dy>dx) {
		temp = dx;
		dx = dy;
		dy = temp;
		xchange = 1;
	} 

	e = ((int)dy<<1) - dx;  
	 
	for (j=0; j<=dx; j++) {
		//video_pt(x,y,c); //640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x; 
		// set pixel color
		//*(char *)pixel_ptr = c;
		VGA_PIXEL(x,y,c);			
		 
		if (e>=0) {
			if (xchange==1) x = x + s1;
			else y = y + s2;
			e = e - ((int)dx<<1);
		}

		if (xchange==1) y = y + s2;
		else x = x + s1;

		e = e + ((int)dy<<1);
	}
}