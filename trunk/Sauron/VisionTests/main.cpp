#include <iostream>
#include <cstdlib>

extern int testBasicImage();
extern int testCamera();
extern int testMarkMapper();
extern int testMarkAssociation();
extern int testVisionModel();

extern int calibrateCamera( int argc, char *argv[] );

int main( int argc, char *argv[] )
{
    int input = 0;
    bool run = true;

    int calibrateNumArgs = 9;
    char *calibrateArgs[] = { "", "-w", "7", "-h", "7", "-s", "2.5", "-n", "15", "\0" };


    while ( run )
    {
        std::cout << "(1)TestCamera  (2)MarkMapper  (3)MarkAssociator  (4)CalibrateCamera  (5)Quit >>";
        std::cin >> input;

        switch ( input )
        {
            case 1:
                testCamera();
                break;

            case 2:
                testMarkMapper();
                break;

            case 3:
                testMarkAssociation();
                break;

            case 4:
                calibrateCamera( calibrateNumArgs, calibrateArgs );
                break;

            case 5:
                run = false;
                break;

            default:
                break;
        }
    }

    return 0;
}