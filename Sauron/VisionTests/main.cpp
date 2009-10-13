#include <iostream>
#include <cstdlib>

extern int testBasicImage();
extern int testCamera();
extern int testMarkMapper();
extern int testMarkAssociation();

int main( int argc, char *argv[] )
{
    //testBasicImage();
    testCamera();
    //testMarkMapper();
    //testMarkAssociation();
    
    return 0;
}