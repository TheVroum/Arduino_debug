#include "ARDUINO.h"


//penser à ajouter ultérieurement un système de mutex
// qui ferait office de pseudo breakpoint interne
// permettant de modifier l'arduino

// Comme la plupart des instructions ne sont pas comoposées et simplement déplacées sur d'autres architecture
// les deux seuls moyens de modifier l'arduino pendant l'exécution du code arduino porté sous windows
// , de manière contrôlées sont de modifier après un temps donné, et de breaker avec la méthode mentionnée ci dessus
// les calls à l'api arduino. Ce que nous feront aussi puisque c'est bien moins aléatoire, bien que limité.

using namespace std;

namespace myArd
{


const std::array <std::string, 5> pinModesString =
{
    "ERROR",
    "OUTPUT      ",//"OUTPUT",
    "INPUT       ",//"INPUT",
    "ERROR",
    "INPUT_PULLUP"
};

myArd::arduino myArd::arduino::defArd;
myArd::eepromClass myArd::eepromClass::eepromStaticMember;
myArd::millisClass myArd::millisClass::millisStaticMember;

const unsigned char digitalPinsRangeLow = 0;
const unsigned char digitalPinsRangeHigh = 13;
const unsigned char analogPinsRangeLow = A0;
const unsigned char analogPinsRangeHigh = A7;

//const size_t controlablePinNumber = 22;//finally as a #define

const float maximumPinTotalDrownCurrent = 0.040;
const float maximumPinIndividualDrownCurrent = 0.020;
const float internalPullupImpedence = 20000.0;
const float internalInputModeImpedence = 10000000.0;
const float internalRailInputModeImpedence = 100000.0;
const float notRailMaximumVoltage = 4.7;
const float notRailMinimumVoltage = 0.3;
const float internalOutputModeImpedence = 10.0;
const float upParasiteVoltage = 6.5;
const float downParasiteVoltage = -1.5;
const float maximumToleratedVoltage = 5.5;
const float minimumToleratedVoltage = -0.5;
const float veryHighResistance = 1000000000.0;
const float internalBaseLeakVoltage = 2.5;

//const size_t myArd:eepromSize = 512;

const float randomFactor = 0.000001;
const float INPUT_PULLUP_VOLTAGE = 4.99;
const float flatVoltageNoise = 0.15;

const float stateImpedences[5] = {std::nanf(""), internalOutputModeImpedence, internalInputModeImpedence, std::nanf(""), internalPullupImpedence};


std::array <bool, /*(const unsigned int)myArd::alterationCallReason*/10> breakingTicks;//1 = break; 0 = don't.
unsigned long callbackInterval = 200;




//pour l'instant elle est déclarée hors classe, plus tard on pourra peut être changer
bool breakAndAlter(alterationCallReason r)
{
#ifndef TWO_INCLUDE_DIRECT_COMPILATION_MODE
static std::mutex mut{};
mut.lock();
    myArd::millisClass::millisStaticMember.pause();
    altererDefArduino(myArd::arduino::defArd, r);
    myArd::millisClass::millisStaticMember.unpause();
mut.unlock();
#endif
#ifndef NDEBUG
    return breakingTicks[r];
#else
    return 0;
#endif
}

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



///implémenter via deux constantes.
void myArd::arduino::digitalWrite(unsigned char pin, pinDigitalValue v)
{
    pinInternalPlugged[pin] = (v == HIGH ? 5.0 : 0.0);
    actualize();
}



///implémenter via une constante
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
    return (pinVoltage[pin]*1024.0)/5.0;
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




unsigned long myArd::millisClass::operator()()
{
    unsigned long long dbtbm_0 = (inClock() * 1000) / CLOCKS_PER_SEC;
    if(myArd::breakAndAlter(myArd::acrFMillis))
        __asm("int $3");
    return dbtbm_0;
}



void myArd::millisClass::startPoint()
{
    substract = clock();
    paused_m = 0;
}


void myArd::millisClass::pause()
{
    assert(!paused_m);
    paused_m = 1;
    substract = clock() - substract;
}


void myArd::millisClass::unpause()
{
    assert(paused_m);
    paused_m = 0;
    substract = clock() - substract;
}


void myArd::millisClass::operator-=(unsigned long t)
{
    substract += t;
}


void myArd::millisClass::operator+=(unsigned long t)
{
    substract -= t;
}


bool myArd::millisClass::paused()
{
    return paused_m;
}


unsigned long long myArd::millisClass::inClock()
{
    if(paused_m)
        return substract;
    else
        return clock() - substract;
}



myArd::eepromClass &EEPROM = myArd::eepromClass::eepromStaticMember;
myArd::millisClass &millis = myArd::millisClass::millisStaticMember;




///S'OCCUPER DE DECLARER TOUT CA AVEC DES CONSTANTES ICI
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
        a = std::make_pair<float, float>(std::nanf(""), 1/0.00000001/*random factor is already here for that*/);
    for(auto &a : pinInternalPlugged)
        a = internalBaseLeakVoltage;
    //for(auto &a : pinVoltage)//set by actualize.
    //currentlyprinted

    actualize();
}



void myArd::arduino::actualize()
{
    if(state != dead)
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
                (static_cast<float>((rand() % 7000) - 1000))/1000.0, 1 / randomFactor));
            std::pair<float/*v*/, float/*c*/> r = calculerResultat(protagonistes);
            if(r.first > notRailMaximumVoltage || r.first < notRailMinimumVoltage)
                protagonistes.push_back(std::pair<float, float>
                (internalBaseLeakVoltage, internalRailInputModeImpedence))
                , r = calculerResultat(protagonistes);
            pinVoltage[i] = r.first;
            pinWorstDrownCurrent[i] = r.second;
        }

        ///TESTS DE SURVIE BASIQUES
        for(auto a : pinVoltage)
            if(a > maximumToleratedVoltage || a < minimumToleratedVoltage)
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
            << " : " << pinVoltage[i] << " volt" << (i < 14 ? "." : " (with ADC).") << std::endl;
        std::cout << "Pin mode : " << pinModesString[pinState[i]] << ".\t" <<
            "External input plugged (voltage in V / impedance in Ohm) : " <<
            pinExternalPlugged[i].first << "V / " << pinExternalPlugged[i].second << "Ohm.\tWORST expectable current through this pin :"
            << pinWorstDrownCurrent[i] << "A." << endl;

    }
        std::cout << "________________________________"
            << std::endl << std::endl << currentlyPrinted << std::endl;
}





void delay(unsigned long ms)
{
    std::this_thread::sleep_for(
        std::chrono::milliseconds(ms));
    if(myArd::breakAndAlter(myArd::acrFDelay))
        __asm("int $3");
}



/*
unsigned long millis()
{
    static clock_t start = (clock() * 1000000) / CLOCKS_PER_SEC;
    clock_t now = (clock() * 1000000) / CLOCKS_PER_SEC;
    unsigned long dbtbm_0 = now - start;
    if(myArd::breakAndAlter(myArd::acrFMillis))
        __asm("int $3");
    return dbtbm_0;
}
*/






void loop();
void setup();



///USAGE : régler les breakpoints, puis sur les fonctions breakées il y aura des variables nommées "dbgVrblToBeModified_xx" -> dbtbm_xx

///ATTENTION : en cas de destruction, les voltages aux bornes de l'arduino ne sont plus actualisés.
/// Le programme pourra éventuellement appeler exit();.

///J'aurais pu, mais ne supporte pas l'activation des pullup à partir de digitalWrite.
/// En effet, cette manière de faire est deprecated par arduino depuis les noubelles MAJ.

///A ajouter : une fonction d'intéractions avec l'utilisateurs lors de chaque call à l'API arduino.
/// Eventuellement, utilisation d'un fichier contentant les réponses à l'avance.





/// /!\ ATTENTION EFFET DE BORD : UN ARDUINO "PRINCIPAL" EST DECLARE EN VARIABLE
/// PRINCIPALE. SA CONSTRUCTION APPELLE ACTUALIZE QUI AFFICHE SUR LA SORTIE STANDARD "COUT".
int main()
{
    if(myArd::breakAndAlter(myArd::acrInit))
        __asm("int $3");

    std::this_thread::sleep_for(
        std::chrono::milliseconds(1));//
    millis.startPoint();
    millis += (CLOCKS_PER_SEC / 1000);//va avec la ligne delay(1) ci dessus. pour simuler les initialisations  de la librairie arduino et éviter d'avoir une millis() qui retourne 0 et qui pourrait camoufler des bugs par exemple

    if(myArd::breakAndAlter(myArd::acrSetup))
        __asm("int $3");
    std::thread th(myArd::caller<decltype(myArd::breakAndAlter), myArd::alterationCallReason>
        , myArd::breakAndAlter, myArd::callbackInterval, true, myArd::acrTime);
    setup();
    for(;;)
    {
        if(myArd::breakAndAlter(myArd::acrLoop))
            __asm("int $3");
        loop();
    }
    return 0;
}
