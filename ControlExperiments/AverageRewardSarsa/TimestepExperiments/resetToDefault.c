#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#define __OSPLATFORM__ "Win32"
#elif __unix__ // all unices
#define __OSPLATFORM__ "Unix"
#elif __posix__
    // POSIX
#elif __linux__
    // linux
#elif __APPLE__
    // Mac OS, not sure if this is covered by __posix__ and/or __unix__ though...
#endif

int main()
{
	printf("Version: %s\n",__VERSION__);
	#ifdef __OSPLATFORM__
	printf("das\n");
	#endif
	return 0;
}
