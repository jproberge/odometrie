/*
test_RED.c
2015-11-18
Public Domain
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include <pigpiod_if2.h>

#include "RED.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"

/*

Nécessite:

Deux encodeurs avec chacun un canal A et un canal B connectés à des 
GPIOs dont le contact commun est connecté au ground.


TO BUILD:

gcc -Wall -pthread -o RED test_RED.c RED.c -lpigpiod_if2

TO RUN:

sudo pigpiod # Seulement si le Daemon n'est pas déjà en fonction.

rosrun odometrie encodeurs


*/

void fatal(char *fmt, ...)
{
   char buf[128];
   va_list ap;

   va_start(ap, fmt);
   vsnprintf(buf, sizeof(buf), fmt, ap);
   va_end(ap);

   fprintf(stderr, "%s\n", buf);

   fflush(stderr);

   exit(EXIT_FAILURE);
}


int optGpioA = 23;
int optGpioB = 24;
int optGpioC = 17;
int optGpioD = 27;
int optGlitch = 1000;
int optSeconds = 0;
int optMode = RED_MODE_DETENT;
char *optHost   = NULL;
char *optPort   = NULL;


void cbf1(int pos)
{
   //printf("Encodeur A : %d\n", pos);
}


void cbf2(int pos)
{
   //printf("Encodeur B : %d\n", pos);
}

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "encodeurs");
   ros::NodeHandle n;
   ros::Publisher encodera_pub = n.advertise<std_msgs::Int32>("encodeur_A", 1000);
   ros::Publisher encoderb_pub = n.advertise<std_msgs::Int32>("encodeur_B", 1000);

   int pi;
   RED_t *renc;   

   if ((optGpioA < 0) || (optGpioB < 0) || (optGpioA == optGpioB))
   {
      fprintf(stderr, "Erreur: les numéros de GPIOs spécifiés pour les encodeurs ne sont pas valides!\n");
      exit(0);
   }
   
   if ((optGpioC < 0) || (optGpioD < 0) || (optGpioC == optGpioD))
   {
      fprintf(stderr, "Erreur: les numéros de GPIOs spécifiés pour les encodeurs ne sont pas valides!\n");
      exit(0);
   }

   pi = pigpio_start(optHost, optPort); /* On se connecte au Pi. */
   

   if (pi >= 0)
   {
      renc = RED(pi, optGpioA, optGpioB, optGpioC, optGpioD, optMode, cbf1, cbf2, &encodera_pub, &encoderb_pub);
      RED_set_glitch_filter(renc, optGlitch);

	  while(ros::ok()) 
	  {
		  sleep(60); // On ne fait rien de particulier, on attend... Les impulsions d'encodeurs susciteront l'appel de callbacks
	  }

      RED_cancel(renc); // On arrête les callbacks

      pigpio_stop(pi); // On se déconnecte du RPI
   }
   return 0;
}

