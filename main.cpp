#include "ARDUINO.h"



using namespace std;







const unsigned char digitalPinsRangeLow = 0;
const unsigned char digitalPinsRangeHigh = 13;
const unsigned char analogPinsRangeLow = A0;
const unsigned char analogPinsRangeHigh = A7;

const size_t controlablePinNumber = 22;

extern const float maximumPinTotalDrownCurrent = 0.40;
extern const float maximumPinIndividualDrownCurrent = 0.20;








void loop();
void setup();




int main()
{
    cout << "Hello World!" << endl;
    return 0;
}
