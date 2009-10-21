/*  
 *  Copyrights:
 *  Ben Pearre, Patrick Mitchell, Marek, Jason Durrie Sept. 2009
 *
 *  This file is part of irobot_create_cu.
 *
 *  irobot_create_cu is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  irobot_create_cu is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with irobot_create_cu. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  If you require a different license, contact Michael Otte at
 *  michael.otte@colorado.edu
 *
 *
 *  This node interacts with Brown's irobot create package, extracting pose
 *  and providing drive commands from the keyboard
 */


#include <assert.h>
#include <curses.h>
#include <errno.h>
#include <math.h>
#include <netdb.h>
#include <sched.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "disp.h"
#include <irobot_create/irobot_create_controller.h>
#include <ros/ros.h>

int old_cursor;
pthread_mutex_t curses_mutex = PTHREAD_MUTEX_INITIALIZER;



IRobotCreateController * controller = (IRobotCreateController*) NULL;
float speed = 0;
float turn = 0;

int
main(int argc, char * argv[]) {
        int cmdchar;

        init_curses();

	ros::init(argc, argv, "prairie_demo");
        controller = new IRobotCreateController();
	ros::NodeHandle n;
	ros::Rate loop_rate(5);

	//srand48(time(NULL));

        int counter = 0;
	while (n.ok()) {
                ++counter;
                cprintf(13,0,"... now at %d", counter);
		cmdchar = getch();
		if (cmdchar != ERR) {
                        cprintf(13, 50, "Hi!");
                        process_key(cmdchar);
		} else {
                        cprintf(13, 60, "Oops");
                }

                if (controller->isBumpedLeft() || controller->isBumpedRight()) {
                        setSpeed(0);
                }

                print_state();
		ros::spinOnce();
		loop_rate.sleep();
	} 

	cleanshutdown(NULL);			/* clean exit */
}


/* Curses printf.  Print the argument at the position specified.
 */
void
cprintf(int row, int col, char * fmt, ...) {
#   ifdef GDB
	return;
#   endif
	static char * buf = NULL;
	int bufsz = 1024;
	int i;
	va_list ap;

	pthread_mutex_lock(&curses_mutex);
	if (!buf) {
		assert(buf = (char*) malloc(bufsz * sizeof(char)));
	}

	va_start(ap, fmt);
	vsnprintf(buf, bufsz, fmt, ap);
	va_end(ap);

	move(row, col);
	for (i = 0; i < strlen(buf); i++) {
		assert(i < bufsz);
		addch(buf[i]);
	}
	clrtoeol();
	refresh();
	pthread_mutex_unlock(&curses_mutex);
}

/* Set up signal handlers.
 */
void
sighandler(int sig) {
	switch(sig) {
	case SIGTERM: case SIGINT:
		cleanshutdown("Received signal %d\n", sig);
                // break;
	default:
		signal(sig, SIG_DFL);
		raise(sig);
	}
}

/* Set up the curses display.  Unless we're running under gdb...
 */
void
init_curses() {
        printf("Initialising curses...\n");
        // Curses stuff
#ifndef GDB    
        initscr();
        keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);
        nonl();
        cbreak();
        noecho();
        old_cursor = curs_set(0);
#endif

        // These are here because this is when we want to make sure
        // that shutdown messages actually show up in a visible
        // fasion.
	signal(SIGSEGV, sighandler);
	signal(SIGABRT, sighandler);
	signal(SIGTERM, sighandler);
	signal(SIGINT, sighandler);
}

/* Move at a speed [0-1] */
float
setSpeed(float newspeed) {
  speed = newspeed;
  controller->setSpeeds(speed, turn);
  return newspeed;
}

float
setTurn(float x) {
  turn = x;
  controller->setSpeeds(speed, turn);
  return x;
}


void
process_key(int cmdchar) {

        cprintf(14, 20, "Got '%d'", cmdchar);

	switch(cmdchar) {

	case 'q':
		cleanshutdown("Shutting down now.");
                break;
        case 'a':
                setSpeed(speed + 0.1);
                break;
        case 's':
                setSpeed(speed - 0.1);
                break;
        case '.':
                setTurn(turn - 0.1);
                break;
        case ',':
                setTurn(turn + 0.1);
                break;
	case ' ': case '0':
		setSpeed(0);
		setTurn(0);
		break;
	}
}

/* Print the current state info to the console
 */
void
print_state() {
        cprintf(14, 0, "Controller at here");

        cprintf(0, 0, "Position [%6.2f %6.2f]   Command speed %2.3f turn %2.3f",
                controller->getX(),
                controller->getY(),
                speed,
                turn);
        cprintf(1, 0, "%s %s", 
                controller->isBumpedLeft() ? "Left " : "     ",
                controller->isBumpedRight() ? "Right " : "     ");
	cprintf(2, 0, "Battery: %5.2fV   %5.2fW   (%f%%) %s",
		controller->getVolts(),
		controller->getWatts(), 
		controller->getPercent(),
		controller->isCharging()?"[Charging]":"            ");
        cprintf(14, 0, "Not yet!  ");
}

/* Shut down everything cleanly.
 */
void
cleanshutdown(char * fmt, ...) {
	va_list ap;

	//pthread_mutex_trylock(&curses_mutex);
        printf("Maybe locked\n");
	curs_set(old_cursor);
        printf("Cursor restored\n");
	nl();
        printf("nl\n");
	endwin();
        printf("Curses has terminated.\n");

	if (fmt) {
		va_start(ap, fmt);
		vfprintf(stderr, fmt, ap);
		fprintf(stderr, "\n");
		va_end(ap);
	}

	printf("\n");
	//printf("%.2f (%.2fh)\n", collision_rate, runtime);
	/*
	assert(logfile = fopen(LOGFILE, "a"));
	fprintf(logfile, "%s\n", log_startup_string);
	*/

        //close(ServerSocket);

	/* FIXME
	if (saver_thread) {
		pthread_join(saver_thread, NULL);
		}*/
	sleep(1);
	exit(0);
}

