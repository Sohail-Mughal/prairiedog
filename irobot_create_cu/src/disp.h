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


#ifndef DISP_INCLUDED
#define DISP_INCLUDED

#define false 0
#define true 1
#define sqr(x) ((x)*(x))
#define max(x,y) ((x)>(y)?(x):(y))
#define min(x,y) ((x)<(y)?(x):(y))

float setSpeed(float newspeed);
float setTurn(float x);

void cleanshutdown(char * msg, ...);
void process_key(int cmdchar);
void print_state();

void cprintf(int row, int col, char * fmt, ...); // dump a string

void * process_sensors_thread(void * foo);
void * send_commands_thread(void * foo);

void init_curses();

void sighandler(int);

#define BUFLEN 2048

// Define GDB to run under the debugger, wherein curses is unhappy

//#define GDB

#ifdef GDB
#  define curs_set(x)
#  define endwin()
#  define initscr()
#  define keypad(x,y)
#  define nodelay(x,y)
#  define nonl()
#  undef getch
#  define getch() 'a'
#  define cbreak()
#  define noecho()
#  undef move
#  define move(x,y)
#  undef addch
#  define addch(x)
#  undef refresh
#  define refresh()
#endif

#endif
