#include <stdio.h>
#include <stdlib.h>

int main()
{
	printf("Version: %s\n",__VERSION__);
	#ifdef __OSPLATFORM__
	printf("das\n");
	#endif
	return 0;
}
