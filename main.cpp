#include "ARDUINO.h"


//penser à ajouter ultérieurement un système de mutex
// qui ferait office de pseudo breakpoint interne
// permettant de modifier l'arduino

// Comme la plupart des instrucitons ne sont pas comoposées et simplement déplacées sur d'autres architecture
// les deux seuls moyens de modifier l'arduino pendant l'exécution du code arduino porté sous windows
// , de manière contrôlées sont de modifier après un temps donné, et de breaker avec la méthode mentionnée ci dessus
// les calls à l'api arduino. Ce que nous feront aussi puisque c'est bien moins aléatoire, bien que limité.

using namespace std;

namespace myArd
{

myArd::arduino myArd::arduino::defArd;
myArd::eepromClass myArd::eepromClass::eepromStaticMember;


const unsigned char digitalPinsRangeLow = 0;
const unsigned char digitalPinsRangeHigh = 13;
const unsigned char analogPinsRangeLow = A0;
const unsigned char analogPinsRangeHigh = A7;

//const size_t controlablePinNumber = 22;//finally as a #define

const float maximumPinTotalDrownCurrent = 0.40;
const float maximumPinIndividualDrownCurrent = 0.20;
const float internalPullupImpedence = 20000.0;
const float internalInputModeImpedence = 10000000.0;
const float internalOutputModeImpedence = 10.0;

//const size_t myArd:eepromSize = 512;

const float randomFactor = 0.01;
const float INPUT_PULLUP_VOLTAGE = 4.99;

const float stateImpedences[4] = {internalOutputModeImpedence, internalInputModeImpedence, std::nanf(""), internalPullupImpedence};



myArd::arduino &myArd::arduino::operator<<(std::string s)
{
    printSentenceAndActualize(s);
    return *this;
}

void myArd::arduino::printSentenceAndActualize(string s)
{
    currentlyPrinted += s;
    actualize();
}



void myArd::arduino::clearAndActualize()
{
    currentlyPrinted.clear();
    actualize();
}




myArd::arduinoState myArd::arduino::getState()
{
    return state;
}


std::array <char, 512> myArd::arduino::getWholeEeprom()
{
    return eeprom;
}





void myArd::arduino::changePluggedOutput(unsigned char pin, float voltage, float impedance)
{
    pinExternalPlugged[pin].first = voltage;
    pinExternalPlugged[pin].second = impedance;
}



void myArd::arduino::setWholeEeprom(std::array <char, 512> input)
{
    eeprom = input;
}





void myArd::arduino::pinMode(unsigned char pin, pinModes m)
{
    pinState[pin] = m;
    actualize();
}

void myArd::arduino::digitalWrite(unsigned char pin, pinDigitalValue v)
{
    pinInternalPlugged[pin] = (v == HIGH ? 5.0 : 0.0);
    actualize();
}


pinDigitalValue myArd::arduino::digitalRead(unsigned char pin)
{
    actualize();
    if(pinVoltage[pin] > 2.5)
        return HIGH;
    else return LOW;
}



int myArd::arduino::analogRead(unsigned char pin)
{
    actualize();
    return pinVoltage[pin];
}




void myArd::arduino::putEeprom(char *outputData, size_t index, size_t size)
{
    assert(outputData);
    assert(size < eepromSize);
    for(unsigned int i = 0; i < size; ++i)
        eeprom[i + index] = outputData[i];
}


void myArd::arduino::getEeprom(char *inputData, size_t index, size_t size)
{
    assert(inputData);
    assert(size < eepromSize);
    for(unsigned int i = 0; i < size; ++i)
        inputData[i] = eeprom[i + index];
}

}







myArd::eepromClass &EEPROM = myArd::eepromClass::eepromStaticMember;






myArd::arduino::arduino()
{
    state = working;
    for(auto &a : eeprom)
        a = 0;
    for(auto &a : pinState)
        a = INPUT;
    for(auto &a : pinWorstDrownCurrent)
        a = 0.000001;
    for(auto &a : pinExternalPlugged)
        a = std::make_pair<float, float>(std::nanf(""), 0.0001);
    for(auto &a : pinInternalPlugged)
        a = 2.5;
    //for(auto &a : pinVoltage)//set by actualize.
    //currentlyprinted

    actualize();
}



void myArd::arduino::actualize()
{
    if(!(state = dead))
    {
        ///ACTUALISATION DES VOLTAGES ET COURRANTS
        for(unsigned int i = 0; i < controlablePinNumber; ++i)
        {
            std::vector <std::pair<float/*voltage*/, float/*impedance*/>>protagonistes;
            protagonistes.push_back(std::pair<float, float>(
                (pinState[i] == INPUT_PULLUP ? INPUT_PULLUP_VOLTAGE : pinInternalPlugged[i])
                , stateImpedences[pinState[i]]));
            protagonistes.push_back(pinExternalPlugged[i]);
            protagonistes.push_back(std::pair<float, float>(
                (static_cast<float>((rand() % 7000) - 1000))/1000.0, randomFactor));
            std::pair<float/*v*/, float/*c*/> r = calculerResultat(protagonistes);
            pinVoltage[i] = r.first;
            pinWorstDrownCurrent[i] = r.second;
        }

        ///TESTS DE SURVIE BASIQUES
        for(auto a : pinVoltage)
            if(a > 5.0 || a < 0.0)
                state = dead;
        float cSum = 0;
        for(auto a : pinWorstDrownCurrent)
        {
            cSum += a;
            if(a > maximumPinIndividualDrownCurrent)
                state = dead;
        }
        if(cSum > maximumPinTotalDrownCurrent)
            state = dead;
    }

    ///AFFICHAGE
    std::cout << std::endl << std::endl << std::endl << std::endl
        << (state == dead ? "Arduino détruit ou endommagé." : "Arduino apparemment fonctionnel.")
        << endl;
    for(unsigned int i = 0; i < controlablePinNumber; ++i)
    {
        std::cout << (i < 14 ? "Pin " : "Pin A") << i % 14
            << (i < 14 ? "" : " (with ADC)")<< " : " << pinVoltage[i] << " volt." << std::endl;
    }
        std::cout << "________________________________"
            << std::endl << std::endl << currentlyPrinted << std::endl;
}





void delay(unsigned long ms)
{
    std::this_thread::sleep_for(
        std::chrono::milliseconds(ms));
}




unsigned long millis()
{
    static clock_t start = (clock() * 1000000) / CLOCKS_PER_SEC;
    clock_t now = (clock() * 1000000) / CLOCKS_PER_SEC;
    return now - start;
}







void loop();
void setup();





///ATTENTION : en cas de destruction, les voltages aux bornes de l'arduino ne sont plus actualisés.
/// Le programme pourra éventuellement appeler exit();.

///J'aurais pu, mais ne supporte pas l'activation des pullup à partir de digitalWrite.
/// En effet, cette manière de faire est deprecated par arduino depuis les noubelles MAJ.

///A ajouter : une fonction d'intéractions avec l'utilisateurs lors de chaque call à l'API arduino.
/// Eventuellement, utilisation d'un fichier contentant les réponses à l'avance.

int main()
{
    setup();
    for(;;)
        loop();
    return 0;
}
