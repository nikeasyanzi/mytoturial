#include "bar.h"

int the_spams;
double the_eggs;

int spam(int new_spams)
{
	    int old_spams = the_spams;
	        the_spams = new_spams;
	            return old_spams;
}

double eggs(double new_eggs)
{
	    double old_eggs = the_eggs;
	        the_eggs = new_eggs;
	            return old_eggs;
}
