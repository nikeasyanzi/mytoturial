#include <stdio.h>
#include <stdlib.h>
#include <stdlib.h>
#if defined(STATIC)||defined (IMPLICITDYNAMIC)
#include "libmyadd.h"
#endif 
#if defined (EXPLICITDYNAMIC)
#include<dlfcn.h>
#endif

int main(int *argc, char **argv)
{
#if defined(STATIC)||defined (IMPLICITDYNAMIC)
	printf("Result = %d\n",myadd(10,20));
#endif

#if defined (EXPLICITDYNAMIC)
	void    *fHandle;
	int     (*func)(int, int); // 注意傳入參數和回傳參數的型態
	fHandle = dlopen("./libmyadd.so",RTLD_LAZY);
	if (!fHandle) {
		fprintf (stderr, "%s\n", dlerror());
		exit(1);

	}
	dlerror();
	func = dlsym(fHandle, "myadd");

	if (func) {        
		printf("Result = %d\n",func(10,20));
	}
	dlclose(fHandle);

/*
	fHandle = dlopen("./libanother.so",RTLD_LAZY);
	if (!fHandle) {
		fprintf (stderr, "%s\n", dlerror());
		exit(1);

	}
	dlerror();
	func = dlsym(fHandle, "myadd");

	if (func) {        
		printf("Result = %d\n",func(10,20));
	}
	dlclose(fHandle);
*/
#endif
	return 0;
}
