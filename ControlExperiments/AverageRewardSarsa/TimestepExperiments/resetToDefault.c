#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#define __OSPLATFORM__ "Win32"
#elif __unix__ // all unices
//Unix
#elif __posix__
#define __OSPLATFORM__ "Posix"
#elif __linux__
#define __OSPLATFORM__ "Linux"
#elif __APPLE__
#define __OSPLATFORM__ "Apple"
#endif

int main()
{
	printf("Version: %s\n",__VERSION__);
	#ifdef __OSPLATFORM__
	printf("OS Used: %s\n",__OSPLATFORM__);
	#endif
	return 0;
}
