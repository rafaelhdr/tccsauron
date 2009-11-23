#include <iostream>
#include <cstdlib>

extern int testBasicImage();
extern int testCamera();
extern int testMarkMapper();
extern int testMarkAssociation();
extern int testVisionModel();

int main( int argc, char *argv[] )
{
    //testBasicImage();
    testCamera();
    //testMarkMapper();
    //testMarkAssociation();
    //testVisionModel();
    
    return 0;
}