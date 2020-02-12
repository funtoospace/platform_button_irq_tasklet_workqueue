/*
 * =====================================================================================
 *
 *       Filename:  platform_button_irq.c
 *
 *    Description:  button irq test application
 *
 *        Version:  1.0
 *        Created:  02/03/2020 08:51:50 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  WL
 *   Organization:  
 *
 * =====================================================================================
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>


int main(int argc, char **argv)
{
	int fd;
	unsigned char key_val;

	fd = open("/dev/platform_button", O_RDWR);
	if (fd < 0)
	{
		printf("can't open /dev/platform_button !\n");
	}

	while (1)
	{
		read(fd, &key_val, 1);
		printf("key_val = 0x%x\n", key_val);
	}

	return 0;
}
