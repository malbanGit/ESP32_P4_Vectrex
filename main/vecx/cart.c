/*
 *  Copyright (C) 2015 Jeroen Domburg <jeroen at spritesmods.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

const unsigned char cart[32768];
int reads=0;
FILE *str=NULL;

void cartInit(char *filename) {
	/*
	FILE *f;
//	printf("cartInit: %s\n", filename);
	if(!(f = fopen(filename, "rb"))){
		perror(filename);
		exit(1);
	}
	fread(cart, 1, sizeof (cart), f);
	fclose(f);
	*/
}

unsigned char cartRead(int addr) 
{
//	reads++;
//	if (addr==4) printf("Reads: %d\n", reads);
	return cart[addr];


}

int cHi;
int parm;
void cartWrite(int addr, unsigned char data) {
	int i;
//	printf("Reads: %i\n", reads);
//	printf("cartWrite: %hhx to addr %x\n", data, addr);
/*	
	if (addr==0) printf("DBG: 0 %02x\n", data);
	if (addr==1) cHi=data<<8;
	if (addr==2) printf("DBG: 1 %04x\n", data|cHi);
	if (addr==0x7ffe) parm=data;
	if ((addr&0xff)==0xff) 
	{
		if (data==1) 
		{
			printf("Unimplemented: multicart\n");
		} 
		else if (data==2) 
		{
			if (str==NULL) 
			{
				str=fopen("vec.bin", "rb");
				if (str==NULL) 
				{
					perror("vec.bin");
					exit(1);
				}
			}
			i=fread(&cart[0x4000], 1, 1024+512, str);
			if (i==0) 
				exit(0);			

			printf("Read %d bytes %hhx.\n", i, cart[0x4000]);
			} 
			else if (data==66) 
			{
				//Do a Doom render...
				;//voomVectrexFrame(parm, &cart[0x1000]);
			}
	}
			*/
}
