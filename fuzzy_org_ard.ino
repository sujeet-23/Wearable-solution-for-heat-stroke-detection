#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE(*_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE(*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE(*_FIS_ARR)(FIS_TYPE*, int, _FIS_ARR_OP);

//***********************************************************************
// Matlab .fis to arduino C converter v2.0.1.25122016                                       
//***********************************************************************

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define TCAADDR 0X70
#include "Adafruit_SHT31.h"
#include <Adafruit_MLX90614.h>

const int GSR = A0;
int sensorValue = 0;
int gsr_average = 0;
int G;
uint32_t tsLastReport = 0;
const int buzzer = 4;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); // Initialize Adafruit MLX90614 object

#define SCREEN_WIDTH 128  // OLED display width,  in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);



//----------
//
void TCAselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


// Number of inputs to the fuzzy inference system
const int fis_gcI = 4;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 72;
int riskLevel = 0; 

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

void displayMessageCentered(String message) {
  int16_t x, y;
  uint16_t w, h;
  oled.getTextBounds(message, 0, 0, &x, &y, &w, &h); // Get text bounds
  // Center the text horizontally and vertically
  x = (SCREEN_WIDTH - w) / 2;
  y = (SCREEN_HEIGHT - h) / 2;
  oled.setCursor(x, y); // Position to display
  oled.println(message); // Text to display
}

// Setup routine runs once when you press reset:
void setup()
{
  pinMode(buzzer, OUTPUT);
  Wire.begin();
//  oled start--------------
  Serial.begin(9600);
  // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }

  delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display

  oled.setTextSize(3);           // larger text size
  oled.setTextColor(WHITE);      // text color
   
//oled end--------------

    TCAselect(1);
    sht31.begin();
    Serial.println("SHT31 test");
    if (!sht31.begin()) { // Set to 0x45 for alternate i2c addr
      Serial.println("Couldn't find SHT31");
    }
    {
     Serial.println("found SHT31");
    }
       
  

long sum = 0;
for (int i = 0; i < 10; i++) { // Average the 10 measurements to remove the glitch
  sensorValue = analogRead(GSR);
  sum += sensorValue;
  delay(5);
}
float averageValue = sum / 10.0; // Calculate the average value
//  Serial.println(gsr_average);

}



// Loop routine runs over and over again forever:
void loop()
{
   
   TCAselect(1);

  float ambientTemp = sht31.readTemperature();
   
    
  TCAselect(1);
   float h = sht31.readHumidity();
    TCAselect(2);
    float objectTemp = mlx.readObjectTempC();
    



    // Calculate core temperature
    float alpha = 0.7665;
    float coreTemp = objectTemp + alpha * (objectTemp - ambientTemp);
    Serial.print("environment Temperature:");
    Serial.println(ambientTemp);
//    Serial.print("Body Temperature:");
    Serial.println(objectTemp);
//    Serial.println();

    sensorValue = analogRead(GSR);
    G = sensorValue - gsr_average;
    float R = ambientTemp + 0.1 * h;
    float H = 100;
//    Serial.println("input");
//    Serial.print("gsr: ");
//    Serial.println(abs(G));
//    Serial.print("Riskfactor: ");
//    Serial.println(R);  
    Serial.print("CoreTemp:");
    Serial.println(coreTemp);
//    Serial.print("hearbeat:");
//    Serial.println(H);
    Serial.println();


//  input-------------------------
    // Read Input: GSR(G)
    g_fisInput[0] = abs(G);
    // Read Input: RiskFactor(R)
    g_fisInput[1] = R;
    // Read Input: Temperature(T)
    g_fisInput[2] = coreTemp;
    // Read Input: Heartbeat(H)
    g_fisInput[3] = H;


//output----------------------------  
    g_fisOutput[0] = 0;
    fis_evaluate();

    riskLevel = g_fisOutput[0];
//  Serial.print("output  ");
//  Serial.println(g_fisOutput[0]);
  Serial.println();

  oled.clearDisplay(); // Clear previous display

  // Display messages based on risk level
  if (riskLevel >= 0 && riskLevel <= 10) {
    displayMessageCentered("");
  } else if (riskLevel >= 11 && riskLevel <= 20) {
    displayMessageCentered("Attention");
  } else if (riskLevel >= 21 && riskLevel <= 30) {
    displayMessageCentered("Warning");
  } else if (riskLevel >= 31 && riskLevel <= 40) {
    displayMessageCentered("Prohibition");
    tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(1000);        // ...for 1 sec
    noTone(buzzer);     // Stop sound...
    delay(1000);
  }

  oled.display(); // Show on OLED
  delay(100);    // Delay for readability, adjust as needed



}

//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Trapezoidal Member Function


FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
    return (FIS_TYPE) min(t1, t2);
}

// S-Shaped membership function
FIS_TYPE fis_smf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1];
    FIS_TYPE m = ((a + b) / 2.0);
    FIS_TYPE t = (b - a);
    if (a >= b) return (FIS_TYPE) (x >= m);
    if (x <= a) return (FIS_TYPE) 0;
    if (x <= m)
    {
        t = (x - a) / t;
        return (FIS_TYPE) (2.0 * t * t);
    }
    if (x <= b)
    {
        t = (b - x) / t;
        return (FIS_TYPE) (1.0 - (2.0 * t * t));
    }
    return (FIS_TYPE) 1;
}

// Pi-shaped Member Function
FIS_TYPE fis_pimf(FIS_TYPE x, FIS_TYPE* p)
{
    return (fis_smf(x, p) * fis_zmf(x, p + 2));
}

// Z-shaped Member Function
FIS_TYPE fis_zmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1];
    FIS_TYPE m = ((a + b) / 2.0);
    FIS_TYPE t = (b - a);
    if (x <= a) return (FIS_TYPE) 1;
    if (x <= m)
    {
        t = (x - a) / t;
        return (FIS_TYPE) (1.0 - (2.0 * t * t));
    }
    if (x <= b)
    {
        t = (b - x) / t;
        return (FIS_TYPE) (1.0 - (2.0 * t * t));
    }
    return (FIS_TYPE) 0;
}


// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions


_FIS_MF fis_gMF[] =
{
    fis_trapmf, fis_smf, fis_pimf, fis_zmf, fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 2, 4, 3, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 4 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { -0.375, 0, 40, 55 };
FIS_TYPE fis_gMFI0Coeff2[] = { 40, 100 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2 };
FIS_TYPE fis_gMFI1Coeff1[] = { 0, 0, 27, 32 };
FIS_TYPE fis_gMFI1Coeff2[] = { 26, 30, 34, 36.5 };
FIS_TYPE fis_gMFI1Coeff3[] = { 34, 35, 37, 39 };
FIS_TYPE fis_gMFI1Coeff4[] = { 37, 39, 50, 50 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3, fis_gMFI1Coeff4 };
FIS_TYPE fis_gMFI2Coeff1[] = { 0, 0, 37, 38.5 };
FIS_TYPE fis_gMFI2Coeff2[] = { 37.5, 38, 39, 41.5 };
FIS_TYPE fis_gMFI2Coeff3[] = { 39, 40.4, 45, 45 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2, fis_gMFI2Coeff3 };
FIS_TYPE fis_gMFI3Coeff1[] = { 0, 0, 88, 110 };
FIS_TYPE fis_gMFI3Coeff2[] = { 88, 100, 175, 190 };
FIS_TYPE fis_gMFI3Coeff3[] = { 175, 190, 220, 220 };
FIS_TYPE* fis_gMFI3Coeff[] = { fis_gMFI3Coeff1, fis_gMFI3Coeff2, fis_gMFI3Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff, fis_gMFI3Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 5, 10 };
FIS_TYPE fis_gMFO0Coeff2[] = { 10, 15, 20 };
FIS_TYPE fis_gMFO0Coeff3[] = { 20, 25, 30 };
FIS_TYPE fis_gMFO0Coeff4[] = { 30, 35, 40 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 1 };
int fis_gMFI1[] = { 2, 2, 2, 2 };
int fis_gMFI2[] = { 2, 2, 2 };
int fis_gMFI3[] = { 2, 2, 2 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2, fis_gMFI3};

// Output membership function set
int fis_gMFO0[] = { 4, 4, 4, 4 };
int* fis_gMFO[] = { fis_gMFO0};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1, 1, 1 };
int fis_gRI1[] = { 2, 1, 1, 1 };
int fis_gRI2[] = { 1, 2, 1, 1 };
int fis_gRI3[] = { 2, 2, 1, 1 };
int fis_gRI4[] = { 1, 3, 1, 1 };
int fis_gRI5[] = { 2, 3, 1, 1 };
int fis_gRI6[] = { 1, 4, 1, 1 };
int fis_gRI7[] = { 2, 4, 1, 1 };
int fis_gRI8[] = { 1, 1, 2, 1 };
int fis_gRI9[] = { 2, 1, 2, 1 };
int fis_gRI10[] = { 1, 2, 2, 1 };
int fis_gRI11[] = { 2, 2, 2, 1 };
int fis_gRI12[] = { 1, 3, 2, 1 };
int fis_gRI13[] = { 2, 3, 2, 1 };
int fis_gRI14[] = { 1, 4, 2, 1 };
int fis_gRI15[] = { 2, 4, 2, 1 };
int fis_gRI16[] = { 1, 1, 3, 1 };
int fis_gRI17[] = { 2, 1, 3, 1 };
int fis_gRI18[] = { 1, 2, 3, 1 };
int fis_gRI19[] = { 2, 2, 3, 1 };
int fis_gRI20[] = { 1, 3, 3, 1 };
int fis_gRI21[] = { 2, 3, 3, 1 };
int fis_gRI22[] = { 1, 4, 3, 1 };
int fis_gRI23[] = { 2, 4, 3, 1 };
int fis_gRI24[] = { 1, 1, 1, 2 };
int fis_gRI25[] = { 2, 1, 1, 2 };
int fis_gRI26[] = { 1, 2, 1, 2 };
int fis_gRI27[] = { 2, 2, 1, 2 };
int fis_gRI28[] = { 1, 3, 1, 2 };
int fis_gRI29[] = { 2, 3, 1, 2 };
int fis_gRI30[] = { 1, 4, 1, 2 };
int fis_gRI31[] = { 2, 4, 1, 2 };
int fis_gRI32[] = { 1, 1, 2, 2 };
int fis_gRI33[] = { 2, 1, 2, 2 };
int fis_gRI34[] = { 1, 2, 2, 2 };
int fis_gRI35[] = { 2, 2, 2, 2 };
int fis_gRI36[] = { 1, 3, 2, 2 };
int fis_gRI37[] = { 2, 3, 2, 2 };
int fis_gRI38[] = { 1, 4, 2, 2 };
int fis_gRI39[] = { 2, 4, 2, 2 };
int fis_gRI40[] = { 1, 1, 3, 2 };
int fis_gRI41[] = { 2, 1, 3, 2 };
int fis_gRI42[] = { 1, 2, 3, 2 };
int fis_gRI43[] = { 2, 2, 3, 2 };
int fis_gRI44[] = { 1, 3, 3, 2 };
int fis_gRI45[] = { 2, 3, 3, 2 };
int fis_gRI46[] = { 1, 4, 3, 2 };
int fis_gRI47[] = { 2, 4, 3, 2 };
int fis_gRI48[] = { 1, 1, 1, 3 };
int fis_gRI49[] = { 2, 1, 1, 3 };
int fis_gRI50[] = { 1, 2, 1, 3 };
int fis_gRI51[] = { 2, 2, 1, 3 };
int fis_gRI52[] = { 1, 3, 1, 3 };
int fis_gRI53[] = { 2, 3, 1, 3 };
int fis_gRI54[] = { 1, 4, 1, 3 };
int fis_gRI55[] = { 2, 4, 1, 3 };
int fis_gRI56[] = { 1, 1, 2, 3 };
int fis_gRI57[] = { 2, 1, 2, 3 };
int fis_gRI58[] = { 1, 2, 2, 3 };
int fis_gRI59[] = { 2, 2, 2, 3 };
int fis_gRI60[] = { 1, 3, 2, 3 };
int fis_gRI61[] = { 2, 3, 2, 3 };
int fis_gRI62[] = { 1, 4, 2, 3 };
int fis_gRI63[] = { 2, 4, 2, 3 };
int fis_gRI64[] = { 1, 1, 3, 3 };
int fis_gRI65[] = { 2, 1, 3, 3 };
int fis_gRI66[] = { 1, 2, 3, 3 };
int fis_gRI67[] = { 2, 2, 3, 3 };
int fis_gRI68[] = { 1, 3, 3, 3 };
int fis_gRI69[] = { 2, 3, 3, 3 };
int fis_gRI70[] = { 1, 4, 3, 3 };
int fis_gRI71[] = { 2, 4, 3, 3 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13, fis_gRI14, fis_gRI15, fis_gRI16, fis_gRI17, fis_gRI18, fis_gRI19, fis_gRI20, fis_gRI21, fis_gRI22, fis_gRI23, fis_gRI24, fis_gRI25, fis_gRI26, fis_gRI27, fis_gRI28, fis_gRI29, fis_gRI30, fis_gRI31, fis_gRI32, fis_gRI33, fis_gRI34, fis_gRI35, fis_gRI36, fis_gRI37, fis_gRI38, fis_gRI39, fis_gRI40, fis_gRI41, fis_gRI42, fis_gRI43, fis_gRI44, fis_gRI45, fis_gRI46, fis_gRI47, fis_gRI48, fis_gRI49, fis_gRI50, fis_gRI51, fis_gRI52, fis_gRI53, fis_gRI54, fis_gRI55, fis_gRI56, fis_gRI57, fis_gRI58, fis_gRI59, fis_gRI60, fis_gRI61, fis_gRI62, fis_gRI63, fis_gRI64, fis_gRI65, fis_gRI66, fis_gRI67, fis_gRI68, fis_gRI69, fis_gRI70, fis_gRI71 };

// Rule Outputs
int fis_gRO0[] = { 1 };
int fis_gRO1[] = { 2 };
int fis_gRO2[] = { 2 };
int fis_gRO3[] = { 2 };
int fis_gRO4[] = { 2 };
int fis_gRO5[] = { 2 };
int fis_gRO6[] = { 2 };
int fis_gRO7[] = { 3 };
int fis_gRO8[] = { 2 };
int fis_gRO9[] = { 3 };
int fis_gRO10[] = { 2 };
int fis_gRO11[] = { 2 };
int fis_gRO12[] = { 2 };
int fis_gRO13[] = { 3 };
int fis_gRO14[] = { 3 };
int fis_gRO15[] = { 3 };
int fis_gRO16[] = { 4 };
int fis_gRO17[] = { 4 };
int fis_gRO18[] = { 4 };
int fis_gRO19[] = { 4 };
int fis_gRO20[] = { 4 };
int fis_gRO21[] = { 4 };
int fis_gRO22[] = { 4 };
int fis_gRO23[] = { 4 };
int fis_gRO24[] = { 2 };
int fis_gRO25[] = { 2 };
int fis_gRO26[] = { 2 };
int fis_gRO27[] = { 2 };
int fis_gRO28[] = { 2 };
int fis_gRO29[] = { 3 };
int fis_gRO30[] = { 3 };
int fis_gRO31[] = { 3 };
int fis_gRO32[] = { 3 };
int fis_gRO33[] = { 3 };
int fis_gRO34[] = { 3 };
int fis_gRO35[] = { 3 };
int fis_gRO36[] = { 3 };
int fis_gRO37[] = { 3 };
int fis_gRO38[] = { 3 };
int fis_gRO39[] = { 3 };
int fis_gRO40[] = { 4 };
int fis_gRO41[] = { 4 };
int fis_gRO42[] = { 4 };
int fis_gRO43[] = { 4 };
int fis_gRO44[] = { 4 };
int fis_gRO45[] = { 4 };
int fis_gRO46[] = { 4 };
int fis_gRO47[] = { 4 };
int fis_gRO48[] = { 4 };
int fis_gRO49[] = { 4 };
int fis_gRO50[] = { 4 };
int fis_gRO51[] = { 4 };
int fis_gRO52[] = { 4 };
int fis_gRO53[] = { 4 };
int fis_gRO54[] = { 4 };
int fis_gRO55[] = { 4 };
int fis_gRO56[] = { 4 };
int fis_gRO57[] = { 4 };
int fis_gRO58[] = { 4 };
int fis_gRO59[] = { 4 };
int fis_gRO60[] = { 4 };
int fis_gRO61[] = { 4 };
int fis_gRO62[] = { 4 };
int fis_gRO63[] = { 4 };
int fis_gRO64[] = { 4 };
int fis_gRO65[] = { 4 };
int fis_gRO66[] = { 4 };
int fis_gRO67[] = { 4 };
int fis_gRO68[] = { 4 };
int fis_gRO69[] = { 4 };
int fis_gRO70[] = { 4 };
int fis_gRO71[] = { 4 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13, fis_gRO14, fis_gRO15, fis_gRO16, fis_gRO17, fis_gRO18, fis_gRO19, fis_gRO20, fis_gRO21, fis_gRO22, fis_gRO23, fis_gRO24, fis_gRO25, fis_gRO26, fis_gRO27, fis_gRO28, fis_gRO29, fis_gRO30, fis_gRO31, fis_gRO32, fis_gRO33, fis_gRO34, fis_gRO35, fis_gRO36, fis_gRO37, fis_gRO38, fis_gRO39, fis_gRO40, fis_gRO41, fis_gRO42, fis_gRO43, fis_gRO44, fis_gRO45, fis_gRO46, fis_gRO47, fis_gRO48, fis_gRO49, fis_gRO50, fis_gRO51, fis_gRO52, fis_gRO53, fis_gRO54, fis_gRO55, fis_gRO56, fis_gRO57, fis_gRO58, fis_gRO59, fis_gRO60, fis_gRO61, fis_gRO62, fis_gRO63, fis_gRO64, fis_gRO65, fis_gRO66, fis_gRO67, fis_gRO68, fis_gRO69, fis_gRO70, fis_gRO71 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0, 0, 60 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 100, 50, 45, 220 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 40 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}





//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************

void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput3[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, fuzzyInput3, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
