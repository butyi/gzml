/*
File: monloader.c (compile: gcc monloader.c -o monloader)

Based on bl08.c (https://github.com/jaromir-sukuba/bl08):
Copyright (c) 2004,2008	Kustaa Nyholm
Copyright (c) 2008 	Robert Larice (SWI return to MON, QY2 chip)
Copyright (c) 2010 	Tormod Volden
Copyright (c) 2013 	Jaromir Sukuba (added A and B ROM routines entry instances, 
			dozen new supported chips, reset polarity)
			http://jaromir.xf.cz/

Copyright (c) 2019 	Janos Bencsik (https://github.com/butyi/)
  Why I write loader for HC908GZ in 2019 while this uC should be already in a museum?
  I still have working hardwares, but I couldn't update the sw because loaders developed
  for WinXP and built in COM port do not work on Win7, Win10 and USB-RS232 devices.
  Moreover, I switched from Windows to Linux several years before.
  Many thanks for my predecessors listed above! They saved my hardwares for further use
  and by this way helped to save green environment (I do not need to produce/buy new device)

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public
License as published by the Free Software Foundation; either
version 2.0 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.
*/

#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <signal.h>


#if defined(__linux__)

# include <sys/wait.h>
# include <sys/ioctl.h> 

#else

// Terrible hack here because POSIX says 'ioctl()' is in <stropts.h> but
// for example Mac OS X Tiger does not have this header, OTOH I think
// that <sys/ioctl.h> is not POSIX either so how do you write
// actual POSIX compliant code that compiles cleanly on POSIX...
extern int ioctl (int filedes, int command, ...);
// End of hack

#endif

#define PIDFILENAME "bl08PIDfile.temp"

#define MEMSIZE 0x10000
#define ROWSIZE 64
#define RAM_PARAM_ADDRESS 0x0080
#define ROUTINES_RAM_ADDRESS 0x0100

char* COM = "/dev/ttyS0"; //default is the first built in port

#include "loader.c"

char version[] = "V0.00 2019.06.29.";

unsigned char scode[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

int com;
int tickP1=15;
int verbose = 1;
int connected=0;
int secured=1;
int eraseFlash=0;
int killPrevious=0;
int dumpStart=-1;
int dumpSize=256;

void comErr(char *fmt, ...) {
	char buf[ 500 ];
	va_list va;
	va_start(va, fmt);
	vsnprintf(buf, sizeof(buf), fmt, va);
	fprintf(stderr,"%s", buf);
	perror(COM);
	va_end(va);
	exit(-1); 
}

void flsprintf(FILE* f, char *fmt, ...) {
	char buf[ 500 ];
	va_list va;
	va_start(va, fmt);
	vsnprintf(buf, sizeof(buf), fmt, va);
	fprintf(f,"%s", buf);
	fflush(f);
	va_end(va);
}
	
void ioctlErrCheck(int e) {
	if (e) {
		flsprintf(stdout,"ioctl returned %d\n",e);
		exit(-1);
	}
}
	
void initSerialPort() {
	com =  open(COM, O_RDWR | O_NOCTTY | O_NDELAY);
	if (com <0) 
		comErr("Failed to open serial port\n");
		
	fcntl(com, F_SETFL, 0);
		
	struct termios opts;
	
	tcgetattr(com, &opts);

	opts.c_lflag  &= ~(ICANON | ECHO | ECHONL | ISIG | IEXTEN);

	opts.c_cflag |= (CLOCAL | CREAD);
	opts.c_cflag &= ~PARENB;
	opts.c_cflag |= CSTOPB; // two stop bits
	opts.c_cflag &= ~CSIZE;
	opts.c_cflag |= CS8;
	
	opts.c_oflag &= ~OPOST;
		
	opts.c_iflag &= ~(INPCK | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);	

	opts.c_cc[ VMIN ] = 0;
	opts.c_cc[ VTIME ] = 10;//0.1 sec
	
	//Unfortunately I couldn't get work non-standard baud rate (14400) on Linux.
	//Therefore I decided to change quarz from 8MHz to 5.33MHz to have standard
	//monitor baudrate. I haven't found 5.33MHz quarz only if I buy 1000 pieces.
	//I could only buy 5.2MHz, which is still inside the tolerance range.
	//This quarz change need to update PLL setup in embedded softwares.
	cfsetispeed(&opts, B9600);   
	cfsetospeed(&opts, B9600);   
	
	if (tcsetattr(com, TCSANOW, &opts) != 0) {
		perror(COM); 
		exit(-1); 
		}
		
	tcflush(com,TCIOFLUSH); // just in case some crap is the buffers
}
	

	
void putByte(unsigned char byte) {
	if (verbose>3)
		flsprintf(stdout,"TX: 0x%02X\n", byte);
	int n = write(com, &byte, 1);
	if (n != 1)
		comErr("Serial port failed to send a byte, write returned %d\n", n);
}
	
int getByte() {
	unsigned char buf;
	int n = read(com, &buf, 1);
	if (verbose>3)
		flsprintf(stdout,n<1?"RX: fail\n":"RX:  0x%02X\n", buf & 0xFF);
	if (n == 1)
		return buf & 0xFF;
	
	comErr("Serial port failed to receive a byte, read returned %d\n", n);
	return -1; // never reached
}

// This reads away break 'character' from the serial line
void flushBreak() { 
	int i;
	for (i=0; i<2; ++i) {
		char buf;
		int n = read(com, &buf, 1);
		if (verbose>3)
			flsprintf(stdout,n<1?"FL: nothing\n":"FL:  0x%02X\n", buf & 0xFF);
	}
}
		
void sendByte(unsigned char byte) {
	putByte(byte);
	unsigned char buf;
	if (read(com, &buf, 1)!=1)
		comErr("Loopback failed, nothing was received\n");
	int rx=buf & 0xFF;
	if (byte !=  rx)
		comErr("Loopback failed, sent 0x%02X, got 0x%02X\n", byte,  rx);
	rx = getByte();
	if (byte !=  rx)
		comErr("Target echo failed, sent 0x%02X, got 0x%02X\n", byte, rx);
}
	
	
void readMemory(unsigned char* p, int addr, int n,int tick) {
	if (verbose>2) 
		flsprintf(stdout,"Read memory address %04X size %04X\n",addr,n);
	sendByte(0x4A); // Monitor mode READ command
	sendByte(addr >> 8);
	sendByte(addr & 0xFF);
	*(p++) = getByte();
	n--;
	int tc=0;
	while (n>0) {
		sendByte(0x1A); // Monitor mode IREAD command
		int b1 = getByte();
		int b2 = getByte();
		*(p++) = b1;
		n--;
		if (n > 0)
			*(p++) = b2;
		n--;
		if (tick) {
			tc++;
			if ((tc & tickP1)==0)
				flsprintf(stdout,".");
		}
	}
}

void writeMemory(unsigned char* p, int addr, int n, int tick) {
	if (verbose>2) 
		flsprintf(stdout,"Write memory address %04X size %04X\n",addr,n);
	sendByte(0x49); // Monitor mode WRITE command
	sendByte(addr >> 8);
	sendByte(addr & 0xFF);
	sendByte(*(p++));
	int tc=1;
	while (n>1) {
		sendByte(0x19); // Monitor mode IWRITE command
		sendByte(*(p++));
		n--;
		if (tick) {
			tc++;
			if ((tc & tickP1)==0)
				flsprintf(stdout,".");
		}
	}
}
	
void connectTarget() {
	int j;
	unsigned char buff[1];
	if (connected)
		return;
  
  if (verbose)
    flsprintf(stdout,"Connect to target.\n");

	flsprintf(stdout, "Security code: ");
	for (j = 0; j<8; ++j) {
		sendByte(scode[j]);
	        flsprintf(stdout, ".");
		}
	flsprintf(stdout, " ");

	flushBreak();
	readMemory(buff, 0x40, 1 , 0);
	connected=1;
	if ((buff[0] & 0x40) == 0){
		flsprintf(stdout,"failed\n");
		secured=1;
	} else {
		flsprintf(stdout,"passed\n");
		secured=0;
	}	
	
}
	

void dumpMemory(unsigned char* p, int addr, int n) {
	int i;
	flsprintf(stdout,"\n");
	for (i = 0; i<n; ++i) {
		if ((i&0xF) == 0)
			flsprintf(stdout,"%04X  ", addr+i);
		flsprintf(stdout,"%02X ", *(p++) & 0xFF);
		if ((i&0xF) == 7)
			flsprintf(stdout," ");
		if ((i&0xF) == 15)
			flsprintf(stdout,"\n");
		}
	if ((i&0xF) != 0)
		flsprintf(stdout,"\n");
}
	
	
int readSP() {
	if (verbose>2) 
		flsprintf(stdout,"Read Stack Pointer\n");
	sendByte(0x0C); // Monitor mode READSP command
	return  (((getByte() << 8) | (getByte() & 0xFF)) - 1) & 0xFFFF;
}	

int runFrom(int PC) {
	unsigned char buff[6];
	int SP=readSP();
	if (verbose>2) 
		flsprintf(stdout,"Execute code PC=%04X SP=%04X\n",PC,SP);
	buff[ 0 ] = 0;//H
	buff[ 1 ] = 0;//CC
	buff[ 2 ] = 0;//A
	buff[ 3 ] = 0;//X
	buff[ 4 ] = PC >> 8;
	buff[ 5 ] = PC & 0xFF;
	writeMemory(buff, SP + 1, 6, 0);
	sendByte(0x28); // Monitor mode RUN command
	return SP;
}	
			
int pressed_key;
int kbhit(void)
{
  struct termios oldt, newt;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  pressed_key = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(pressed_key != EOF)
  {
    //ungetc(pressed_key, stdin);
    return 1;
  }
 
  return 0;
}

int readSrec(int verbose,FILE* sf,unsigned char* p, unsigned int size, unsigned int base, int* ranges, unsigned int rn) {
	if (verbose)
		flsprintf(stdout,"Reading S-records\n");
	memset(p,0xff,size);
	unsigned char line[2+2+255*2+2+1]; // Sx + count + 255 bytes for data address & checksum + CR/LF +nul (in windows)
	unsigned int amax=0;
	unsigned int rc=0;
	while (fgets(line,sizeof(line),sf)!=NULL) {
	   unsigned int o=0;
	   if (line[0]=='S') {
			unsigned int n,a;
			sscanf(line+2,"%2x",&n);
			n--;
			if (line[1]=='1') {
				sscanf(line+4,"%4x",&a);
				n=n-2;
				o=8;
			}
			if (line[1]=='2') {
				sscanf(line+4,"%6x",&a);
				n=n-4;
				o=10;
			}
			if (line[1]=='3') {
				sscanf(line+4,"%8x",&a);
				n=n-6;
				o=12;
			}
			if (o!=0) {
				unsigned int i,j;
				if (ranges) {
				    for (i=0; i<rc; i+=2) {
						unsigned int rlo=ranges[i];
						unsigned int rhi=rlo+ranges[i+1];
						if (!((a+n<=rlo) || (rhi<=a))) {
							flsprintf(stderr,"Overlapping S-record ranges %04X,%04X and %0x4 %04X\n",rlo,rhi-rlo,a,n);
							exit(-1);
							}
						}
					if (rc + 2 >= rn) 
						return -1;
					ranges[rc]=a;
					ranges[rc+1]=n;
					rc += 2;
					
					unsigned int cf=0;
					do compact: {
						for (i=0; i<rc; i+=2) {
							for (j=i+2; j<rc; j+=2) {
								cf=1;
								if (ranges[i]+ranges[i+1]==ranges[j])
									ranges[i+1] += ranges[j+1];
								else if (ranges[i]==ranges[j]+ranges[j+1])
									ranges[i]-=ranges[j+1];
								else 
									cf=0;
								if (cf) {
									for (i=j+2; i<rc; i++)	
										ranges[i]=ranges[i+2];
									rc-=2;
									cf=0;
									goto compact;
									}
								}
							}
						} while (cf);
					}
				for (i=0; i<n; ++i) {
					unsigned int d;
					sscanf(line+o+i*2,"%2x",&d);
					if ( (a >= base) && (a < base+size)) {
						p[ a - base ] = d;
						a++;
						amax = a>amax ? a : amax;
						}
					}
				}
			}
		if (verbose>1)
			flsprintf(stdout,">>> %s",line);
		if (verbose && o==0)
			flsprintf(stdout,"Line ignored: %s\n",line);
		}
	if (verbose) {
		if (ranges) {
			unsigned int i;
			for (i=0; i<rc; i+=2) 
				flsprintf(stdout,"S-record data address %06X size %06X\n",ranges[i],ranges[i+1]);
			}
		flsprintf(stdout,"\n");
		}
	return rc;
}

void printHelp() {
		flsprintf(stdout,"monloader - MC68HC908GZ60 Monitor Loader - version %s\n",version);
		flsprintf(stdout,"Download software into flash memory from an S19 file using monitor mode\n");
		flsprintf(stdout,"Usage: \n");
		flsprintf(stdout," monloader [-p:d:l:ehkv:qs:] [filename.s19]\n");
		flsprintf(stdout,"  -p port        Set serial com PORT used to communicate with target\n"); 
		flsprintf(stdout,"                 (default '/dev/ttyS0')\n");
		flsprintf(stdout,"  -d address     DUMP from memory address\n");
		flsprintf(stdout,"  -l length      Dump LENGTH, default 0x80\n");
		flsprintf(stdout,"  -e             ERASE only the target using mass erase, clearing security bytes\n");
		flsprintf(stdout,"  -h             Print out this HELP text\n");
		flsprintf(stdout,"  -k             KILL previous instance of monloader\n");
		flsprintf(stdout,"  -v verbosity   Set VERBOSITY level, valid values are 0-4, default 1\n");
		flsprintf(stdout,"  -q             Run QUIETly, same as -l 0\n");		
		flsprintf(stdout,"  -s string      Security code, as a string of hex bytes separated by space or colon\n");
		flsprintf(stdout,"                 (default 'FF FF FF FF FF FF FF FF')\n");
		flsprintf(stdout,"\n");
		flsprintf(stdout,"Examples, prefered using procedure:\n");
		flsprintf(stdout,"  ./monloader -e -> Erase the whole flash memory to have empty uC.\n");
		flsprintf(stdout,"  ./monloader hmbl.s19 -> Download hmbl.s19 software into empty uC with verify.\n");
		flsprintf(stdout,"\n");
		flsprintf(stdout,"  ./monloader -d 0xFF00 -> Check if vectors were written properly.\n");
		flsprintf(stdout,"Baud rate is fixed 9600. This needs 5.333Mhz quarz, but 5.2MHz is still sufficient.\n");
		flsprintf(stdout,"\n");
	exit(0);
}

int getIntArg(char* arg) {
	if (strlen(arg)>=2 && memcmp(arg,"0x",2)==0) {
		unsigned int u;
		sscanf(arg+2,"%X",&u);
		return u;
	}
	else 
	{
		int d;
		sscanf(arg,"%d",&d);
		return d;
	}
}

void setSecurityCode(char* str) {
	int i;
	int consumed;
	for (i=0; i<8 && *str; ++i) {
		if (sscanf(str," %2hhx%n",&scode[i],&consumed)!=1) {
			flsprintf(stderr,"Bad security code: %s\n",str);
			abort();
		}
		str += consumed;
		while (*str==' ') str++; /* tolerate spaces */
		if (*str==':') str++;    /* and one colon */
	}
	if (i<8) {
		flsprintf(stderr,"Found only %d bytes in security code\n",i);
		abort();
	}
}
	

void parseArgs(int argc, char *argv[]) {	
	int c;
	while ((c = getopt (argc, argv, "p:d:l:ehkv:qs:")) != -1) {
		switch (c) {
			case 'p' ://port 
				COM=optarg;
				break;
			case 'd' ://dump
				dumpStart=getIntArg(optarg);
				break;
			case 'l' ://length
				dumpSize=getIntArg(optarg);
				break;
			case 'e' ://erase
				eraseFlash = 1;
				break;
			case 'h' ://help
				printHelp();
				break;
      case 'k' ://kill
				killPrevious = 1;
        break;
			case 'v' ://verbose
				sscanf(optarg,"%d",&verbose); 
				break;
			case 'q' ://quiet
				verbose=0;
				break;
			case 's' ://security key
				setSecurityCode(optarg);
				break;
			case '?' :
				if (isprint (optopt))
					flsprintf(stderr,"Unknown option `-%c'.\n", optopt);
				else
					flsprintf(stderr,"Unknown option character `\\x%x'.\n",optopt);
			  default:
				flsprintf(stderr,"Bug, unhandled option '%c'\n",c);
				exit(-1);
			}
		}
	if (argc<=1) 
		printHelp();
}


void requestReset() {
  flsprintf(stdout,"Please do a power reset on the target uC and press any key!\n");
  while(!kbhit())
    sleep(1);
}

void deletePidFile() {
	int stat=remove(PIDFILENAME);
	if (stat)
		flsprintf(stderr,"remove returned %d\n",stat);
}
	
void killPreviousInstance() {
	atexit(deletePidFile);
	int pid;
	FILE* pidf=fopen(PIDFILENAME,"r");
	if (pidf) {
		fscanf(pidf,"%d",&pid);
		int stat=kill(pid,SIGKILL);
		if (stat!=0)
			flsprintf(stderr,"kill returned %d\n",stat);
			
		fclose(pidf);
		waitpid(pid,&stat,0);
		if (WIFEXITED(stat)==0)
			flsprintf(stderr,"waitpid returned %d\n",WIFEXITED(stat));
		}
	pidf=fopen(PIDFILENAME,"w");
	fprintf(pidf,"%d\n",getpid());
	fclose(pidf);
}

void selectFlash(unsigned char* buff, unsigned int address, unsigned int length){
  if(ROWSIZE<length){
    flsprintf(stderr,"Row length %X is larger than %X from address %04X\n", length, ROWSIZE, address);
    exit(-1);
	}		
  if(0x8000 <= address){
    //  FL1CR FF88 
    //  FL1BPR FF80
    //p_flcr  (PGM 0x01, ERASE 0x02, MASS 0x04, HVEN 0x08)
    buff[0]=0xFF; 
    buff[1]=0x88; 
    //p_flbpr
    buff[2]=0xFF; 
    buff[3]=0x80; 
  } else {
    //  FL2CR FE08 
    //  FL2BPR FF81
    //p_flcr  (PGM 0x01, ERASE 0x02, MASS 0x04, HVEN 0x08)
    buff[0]=0xFE; 
    buff[1]=0x08; 
    //p_flbpr
    buff[2]=0xFF; 
    buff[3]=0x81; 
  }
  //address
  buff[4]=(address>>8)&0xFF;
  buff[5]=address&0xFF;

  //ret
  buff[6]=0xEE; //default value: error

  //len
  if(ROWSIZE<length){
    flsprintf(stderr,"Length to download at address %04X is too long %d! Limited to %d.\n", length, ROWSIZE);
    length=ROWSIZE;//limit length
  }
  buff[7]=length;
  //flsprintf(stdout,"  Address %04X\n",address);
}

void checkJumpBackToMon(){
  // SWI drops back into MON, which will send a BREAK
  unsigned char buf;
  if (read(com, &buf, 1) != 1)//wait for character
    comErr("ERROR: waiting for MON, nothing was received\n");
  if(buf != 0)//check if it was break
    comErr("ERROR: unexpected swi answer read %x\n", buf);
}

void massErase(unsigned int address){
  unsigned char buff[8];
  selectFlash(buff, address, 0);

	//write parameters to RAM
	writeMemory(buff, RAM_PARAM_ADDRESS+ROWSIZE, 8, 0);
  
  //readMemory(buff, RAM_PARAM_ADDRESS+ROWSIZE, 8, 0);
  //dumpMemory(buff, RAM_PARAM_ADDRESS+ROWSIZE, 8);

  //run erase subroutine
  runFrom(MASS_ERASE_ADDRESS); //call mass erase 
  
  checkJumpBackToMon();

  //check result
  readMemory(buff, RAM_PARAM_ADDRESS+ROWSIZE, 8, 0);
  if(0 == buff[6]){
    flsprintf(stdout,"  Mass erase was successful.\n");
  } else {
    flsprintf(stdout,"  Mass erase failed with error code %02X\n",buff[6]);
  }

}


void downloadRow(unsigned char* datasource, unsigned int address, unsigned int length){  
  unsigned char buff[8];
  selectFlash(buff, address, length);

	//write data and parameters to RAM
	writeMemory(&datasource[address & 0xFFC0], RAM_PARAM_ADDRESS, ROWSIZE, 0);
	writeMemory(buff, RAM_PARAM_ADDRESS+ROWSIZE, 8, 0);//2+2+2+1+1

  //unsigned char readout_image[64+7];
  //readMemory(readout_image, RAM_PARAM_ADDRESS, ROWSIZE+8, 0);
  //dumpMemory(readout_image, RAM_PARAM_ADDRESS, ROWSIZE+8);

  //run write row subroutine
  runFrom(WRITE_ROW_ADDRESS); //call burn
  
  checkJumpBackToMon();

  //check result
  readMemory(buff, RAM_PARAM_ADDRESS+ROWSIZE, 8, 0);
  if(0 == buff[6]){
    flsprintf(stdout,"  Download row %04X was successful.\n",address);
  } else {
    flsprintf(stdout,"  Download row %04X failed with error code %02X\n",address,buff[6]);
  }

}


int main(int argc, char *argv[]) {	
  unsigned char s19_image[ MEMSIZE ]; // S19 file image
  unsigned char readout_image[ MEMSIZE ]; // HC908 memory image
	int maxrc=256;
	int ranges[maxrc];
	int rc=0;
	unsigned int i,x;

	parseArgs(argc,argv);
	
	if (killPrevious)
		killPreviousInstance();
	
	if (verbose)
		flsprintf(stdout,"monloader - MC68HC908GZ60 Monitor Loader - version %s\n",version);
	
	initSerialPort();
	requestReset();
	
	if (eraseFlash) {
    if (verbose)
      flsprintf(stdout,"Mass erase the whole flash memory.\n");
		connectTarget();
    if (verbose)
      flsprintf(stdout,"Load flash handler routines into RAM.\n");
  	writeMemory(ram_routines, ROUTINES_RAM_ADDRESS, sizeof(ram_routines), 0); 
    if (verbose)
      flsprintf(stdout,"Mass erase of Flash 1 memory.\n");
		massErase(0xFF80);//Flash 1, address of FL1BPR to do mass erase even though security has not passed 
    if(secured){
      flsprintf(stdout,"NOTE! Mass erase of Flash 2 is skipped because security failed. Do erase command again to erase Flash 2 too with passed security.\n");
    } else {
      if (verbose)
        flsprintf(stdout,"Mass erase of Flash 2 memory.\n");
	  	massErase(0x1000);//Flash 2
    }
    if (verbose)
      flsprintf(stdout,"Done.\n");
		return 0;
	}

  connectTarget();
  if(secured){
    flsprintf(stderr,"Security has been failed, command cannot be executed. Give the proper security key or do a mass erase.\n");
    exit(-1);
  }

  
	if (0<=dumpStart) {
    if (verbose)
      flsprintf(stdout,"Dump memory from %04X len %X.\n",dumpStart,dumpSize);
		readMemory(readout_image, dumpStart, dumpSize, verbose);
    dumpMemory(readout_image, dumpStart, dumpSize);
    if (verbose)
      flsprintf(stdout,"Done.\n");
		return 0;
	}

  
  //load S19, if there is  
	for (i=optind; i<argc; i++) {
		char* filename=argv[i];
		FILE* sf = fopen(filename, "r");
		if (sf == NULL) { 
			flsprintf(stderr,"Failed to open '%s'\n", filename);
			exit(-1);
		}
		
	  memset(s19_image,0xFF,sizeof(s19_image));
		int rn=readSrec(verbose , sf , s19_image , sizeof(s19_image) , 0x0000 , ranges+rc , maxrc - rc);
		if (rn<0) {
			flsprintf(stderr,"Too many discontinuous data ranges in S-record file '%s'\n",filename);
			exit(-1);
		}
		rc += rn;		
		fclose(sf);
  }
  if(0<rc){//if there is data to download
    unsigned int r, address, lastaddress, len;
    unsigned char useable_flash[MEMSIZE];
	  

    //test code, set data to be downloaded manually
    //memset(s19_image,0xFF,sizeof(s19_image));
    //memset(&s19_image[0x1000],0xA5,2);
    //memset(&s19_image[0xFFFC],0xA5,2);


	  //set up valid areas for check s19 address ranges later
	  memset(useable_flash,0,sizeof(useable_flash));
    for (i=0; i<MEMSIZE; i++ ){
      //Flash 1 ($8000–$FDFF, $FFCC–$FFFF)
      if(0x8000<=i && i<=0xFDFF)useable_flash[i]=1;
      if(0xFFCC<=i && i<=0xFFFF)useable_flash[i]=1;
      //Flash 2 ($0462–$04FF, $0980–$1B7F, $1E20–$7FFF)
      if(0x0462<=i && i<=0x04FF)useable_flash[i]=1;
      if(0x0980<=i && i<=0x1B7F)useable_flash[i]=1;
      if(0x1E20<=i && i<=0x7FFF)useable_flash[i]=1;
    }
    
    //go through on every row to check if address is valid
    x=0;
    for (r=0; r<MEMSIZE; r+=ROWSIZE ) {
      //check if data to be programmed is in a valid flash range
      for (i=0; i<ROWSIZE; i++ ){
        if(
            (s19_image[r+i] != 0xFF) && //address to be programmed and
            (!useable_flash[r+i]) //address is not valid
        ){
          flsprintf(stderr,"Address in S-record file %04X is not a valid flash address for MC68HC908GZ60!\n",r+i );
          x++;
        }
      }
    }
    if(x)exit(-1);


    if (verbose)
      flsprintf(stdout,"Load flash handler routines into RAM.\n");
  	writeMemory(ram_routines, ROUTINES_RAM_ADDRESS, sizeof(ram_routines), 0); 

    //go through on every row to program
    for (r=0; r<MEMSIZE; r+=ROWSIZE ) {
      address=0;
      //search first used byte of row
      for (i=0; i<ROWSIZE; i++ ) {
        if(s19_image[r+i] != 0xFF){
          address=r+i;
          break;
        }
      }
      //search last used byte of row
      lastaddress=0;
      for (i=ROWSIZE; i; i-- ) {
        if(s19_image[r+i-1] != 0xFF){
          lastaddress=r+i-1;
          break;
        }
      }
      
      //if row contains data to program
      if(address && lastaddress){
        len = lastaddress - address + 1;
        if (verbose)
          flsprintf(stdout,"Write row %04X from %04X to %04X (len=%d).\n",r,address,lastaddress,len);
        downloadRow(s19_image,address,len);
        if (verbose)
          flsprintf(stdout,"  Read data back.\n");
        readMemory(readout_image, address, len, 0);
        //dumpMemory(readout_image, address, len);
        x=0;
        for(i=0;i<len;i++){
          if(readout_image[i] != s19_image[address+i]){
            flsprintf(stderr,"  Verify error at address %04X! Should be %02X, but it is %02X.\n", address+i, s19_image[address+i], readout_image[i] );
            x++;
          }
        }
        if(x){
          flsprintf(stderr,"Exit due to %d verify errors.\n", x );
          exit(-1);
        }  
        if (verbose)
          flsprintf(stdout,"  Verify passed.\n");
          
      }
    }

    if (verbose)
      flsprintf(stdout,"Download done.\n");

  }

	return 0;
}





