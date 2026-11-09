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


// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(((( signed long long)(a))<< 15) / (b)) 
#define sqrtfix(a) (float2fix15(sqrt(fix2float15(a))))

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
typedef struct list {
   fix15 pos_x;
   fix15 pos_y;
   fix15 vx; 
   fix15 vy ;
   int color; 
} Boid;



//!========================================================================
//! NUMBER OF BOIDS
//!========================================================================
#define NUM_BOIDS 3
Boid boid_list[NUM_BOIDS];
//!========================================================================

void boid_initialize(Boid* b) {
    b->pos_x = 0;
    b->pos_y = 0;
    b->vx = 0;
    b->vy = 0;
    b->color = white;
}

// bound type 
fix15 old_bound_type = 0;
fix15 bound_type = 0;


//*========================================================================
//* Creating Multiple Boids
//*========================================================================
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Start in frame with random location
  *x = int2fix15(rand() % (540-100) + 100);
  *y = int2fix15(rand() % (380-100) + 100);

  // random total speed
  float random_speed = (float)rand()/(float)(RAND_MAX) * (6-3) + 3;
  float random_angle = (float)rand()/(float)(RAND_MAX) * (2*PI);

  *vx = float2fix15(random_speed *  (float) cos ((double) random_angle));
  *vy = float2fix15(random_speed *  (float) sin ((double) random_angle));

}


//*========================================================================
//* Updating Boid velocities per interaction with boundaries
//*========================================================================
static inline void boundary_check(int boid_num) {
  fix15* x = &boid_list[boid_num].pos_x;
  fix15* y = &boid_list[boid_num].pos_y;
  fix15* vx = &boid_list[boid_num].vx;
  fix15* vy = &boid_list[boid_num].vy;

  // If the boid is near an edge, make it turn by turnfactorWW
  if (hitTop(*y)) {
      *vy = *vy + turnfactor ;
  }
  if (hitBottom(*y)) {
      *vy = *vy - turnfactor ;
  } 
  if (hitRight(*x)) {
      *vx = *vx - turnfactor;
  }
  if (hitLeft(*x)) {
      *vx = *vx + turnfactor;
  } 

}


//*========================================================================
//* Updating Each Boid
//*========================================================================
void boid_update(int cur)
{
    fix15* x = &boid_list[cur].pos_x;
    fix15* y = &boid_list[cur].pos_y;
    fix15* vx = &boid_list[cur].vx;
    fix15* vy = &boid_list[cur].vy;
    
    fix15 xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0,  close_dx = 0, close_dy = 0;
    fix15 neighboring_boids = 0;
    int i;
    for (i = 0; i < 50; i++) {
      if(i != cur) {
      fix15* x_o = &boid_list[i].pos_x;
      fix15* y_o = &boid_list[i].pos_y;
      fix15* vx_o = &boid_list[i].vx;
      fix15* vy_o = &boid_list[i].vy;
      // Compute differences in x and y coordinates
      fix15 dx = *x - *x_o;
    //   if(dx > int2fix15(320)) {dx = dx - int2fix15(640);}
    //   if(dx < int2fix15(-320)) {dx = dx + int2fix15(640);}
      fix15 dy = *y - *y_o;
    //   if(dy > int2fix15(240)) {dy = dy - int2fix15(480);}
    //   if(dy < int2fix15(-240)) {dy = dy + int2fix15(480);}

      // Are both those differences less than the visual range?
      if (dx < visualRange && dy < visualRange && dx > -visualRange && dy > -visualRange) {
          // If so, calculate the squared distance
          fix15 squared_distance = multfix15(dx,dx) + multfix15(dy,dy);

          // Is squared distance less than the protected range?
          if (squared_distance < multfix15(protectedRange,protectedRange)) { 

              // If so, calculate difference in x/y-coordinates to nearfield boid
              close_dx += dx;
              close_dy += dy;
          }

          // If not in protected range, is the boid in the visual range?
          else if (squared_distance < multfix15(visualRange,visualRange)) {

              // Add other boid's x/y-coord and x/y vel to accumulator variables
              xpos_avg += *x_o;
              ypos_avg += *y_o;
              xvel_avg += *vx_o;
              yvel_avg += *vy_o;

              // Increment number of boids within visual range
              neighboring_boids += int2fix15(1);
          }
        }
      }    
    }

    // If there were any boids in the visual range . . .            
    if (neighboring_boids > 0) {

        // Divide accumulator variables by number of boids in visual range
        // xpos_avg = divfix(xpos_avg,neighboring_boids);
        // ypos_avg = divfix(ypos_avg,neighboring_boids);
        // xvel_avg = divfix(xvel_avg,neighboring_boids);
        // yvel_avg = divfix(yvel_avg,neighboring_boids);

        xpos_avg = float2fix15( fix2float15(xpos_avg) / fix2float15(neighboring_boids) );
        ypos_avg = float2fix15( fix2float15(ypos_avg) / fix2float15(neighboring_boids) );
        xvel_avg = float2fix15( fix2float15(xvel_avg) / fix2float15(neighboring_boids) );
        yvel_avg = float2fix15( fix2float15(yvel_avg) / fix2float15(neighboring_boids) );
        
        // Add the centering/matching contributions to velocity
        *vx = *vx + multfix15((xpos_avg - *x),centeringfactor) +  multfix15((xvel_avg - *vx), matchingfactor);                      

        *vy = *vy + multfix15((ypos_avg - *y),centeringfactor) +  multfix15((yvel_avg - *vy), matchingfactor);
    } 

    //  printf("%d \n", fix2int15(*vx));
    // Add the avoidance contribution to velocity
    *vx = *vx + multfix15(close_dx,avoidfactor);
    *vy = *vy + multfix15(close_dy,avoidfactor);

    boundary_check(cur);


     
    // If the boid has a bias, bias it!
    // biased to right of screen
    if (cur >= 0 && cur < NUM_BOIDS/2) {
        *vx = multfix15((int2fix15(1) - biasval),(*vx)) + biasval;
    }
        
    // biased to left of screen
    else if (cur >= 25 && cur < NUM_BOIDS) {
        *vx = multfix15((int2fix15(1) - biasval),(*vx)) - biasval;
    }

       

    // Calculate the boid's speed
    // Slow step! Lookup the "alpha max plus beta min" algorithm
    fix15 temp_add = multfix15(*vx,*vx) + multfix15(*vy,*vy);
    fix15 speed = sqrtfix( temp_add );

    // printf("%d \n", fix2int15(speed));
    // // Enforce min and max speeds
    if (speed < minspeed && speed != 0) {
         *vx = multfix15(divfix(*vx,speed),minspeed);
         *vy = multfix15(divfix(*vy,speed),minspeed);
    }

    if (speed > maxspeed) {
         *vx = multfix15(divfix(*vx,speed),maxspeed);
         *vy = multfix15(divfix(*vy,speed),maxspeed);
    }


    // Update position using velocity
    *x = *x + *vx ;
    if(*x > int2fix15(640)) {
		*x = *x - int2fix15(15);
		*vx = -*vx;
	}
    if(*x < int2fix15(0)) {
		*x = *x + int2fix15(15);
		*vx = -*vx;
	}
	
    *y = *y + *vy ;
    if(
		*y > int2fix15(480)) {*y = *y - int2fix15(15);
		*vy = -*vy;
	}
    if(
		*y < int2fix15(0)) {*y = *y + int2fix15(15);
		*vy = -*vy;
	}
    
}




int main(void)
{
  	
	
    int i;
    for(i = 0; i < NUM_BOIDS; i++) {
        boid_initialize(&boid_list[i]);
        // spawnBoid(&boid_list[i].pos_x, &boid_list[i].pos_y, &boid_list[i].vx, &boid_list[i].vy);
    }
    
    boid_list[0].pos_x = int2fix15(200);
    boid_list[0].pos_y = int2fix15(200);
    boid_list[0].vx = int2fix15(4);
    boid_list[0].vy = int2fix15(4);

    boid_list[1].pos_x = int2fix15(208);
    boid_list[1].pos_y = int2fix15(205);
    boid_list[1].vx = int2fix15(4);
    boid_list[1].vy = int2fix15(4);  

    boid_list[2].pos_x = int2fix15(210);
    boid_list[2].pos_y = int2fix15(212);
    boid_list[2].vx = int2fix15(4);
    boid_list[2].vy = int2fix15(4);  

    int count;
    count = 0;
    printf("iteration number is %d\n", count);
    int j;
    for(j = 0; j < NUM_BOIDS; j++) {
        printf("boid number: %d\n", j);
        printf("%f \n",fix2float15(boid_list[j].pos_x));
        printf("%f \n",fix2float15(boid_list[j].pos_y));
        printf("%f \n",fix2float15(boid_list[j].vx));
        printf("%f \n\n",fix2float15(boid_list[j].vy));
    }

    
	for(count = 0; count != 5; count++)
	{
        printf("iteration number is %d\n", count+1);
        for (i = 0; i < NUM_BOIDS; i++) {
            int x = fix2int15(boid_list[i].pos_x);
            int y = fix2int15(boid_list[i].pos_y);
            
            //? Update boid's position, velocity, and behavior using the boid_update function
            boid_update(i);

            //? Get the updated position of the boid
            x = fix2int15(boid_list[i].pos_x);
            y = fix2int15(boid_list[i].pos_y);

        }

        j = 0;
        for(j = 0; j < NUM_BOIDS; j++) {
            printf("boid number: %d\n", j);
            printf("%f \n",fix2float15(boid_list[j].pos_x));
            printf("%f \n",fix2float15(boid_list[j].pos_y));
            printf("%f \n",fix2float15(boid_list[j].vx));
            printf("%f \n",fix2float15(boid_list[j].vy));
        }

        //? Set frame rate by waiting for a fixed time (17 ms) before next iteration
		usleep(10700);

		
	} // end while(1)


    
} // end main

