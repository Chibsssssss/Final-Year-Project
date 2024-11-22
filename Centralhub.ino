#define BLYNK_TEMPLATE_ID "TMPL2_Rjbd6Ir"
#define BLYNK_TEMPLATE_NAME "Home Automation FYP"
#define BLYNK_AUTH_TOKEN "QAyM-9jIjv7SjSduBBzaLBxN4ingVKkL"
// Fuzzy Logic ESP32 Code

#include <WiFi.h>
#include <esp_now.h>
#include "fis_header.h"

// Replace with your network credentials
char ssid[] = "Prof";
char pass[] = "nzube123";
char auth[] = "QAyM-9jIjv7SjSduBBzaLBxN4ingVKkL";

// Number of inputs to the fuzzy inference system
const int fis_gcI = 6;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 4;
// Number of rules to the fuzzy inference system
const int fis_gcR = 20;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

// Structure to receive sensor data (must match Peripheral ESP32)
typedef struct struct_message {
  float temp;
  float light;
  bool motion;
  int gas_level;
  int hour;
  float current;
} struct_message;

// Create a struct_message
struct_message sensorData;

// Peer MAC address of Peripheral ESP32 (replace with actual address)
uint8_t broadcastAddress[] = { 0xa0, 0xa3, 0xb3, 0x2f, 0x93, 0x00 };

// Array to store actuator commands
float actuatorCommands[4] = { 0 };  // You might need to adjust the size based on your FIS outputs

// Callback function when data is received
void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));
  Serial.println("Received sensor data:");
  Serial.printf("Temp: %.2f, Light: %.2f, Motion: %d, Gas: %d, Current: %.2f, hour: %d\n",
                sensorData.temp, sensorData.light, sensorData.motion,
                sensorData.gas_level, sensorData.current, sensorData.hour);

  // Assign received sensor data to FIS inputs
  g_fisInput[0] = sensorData.temp;       // Temperature
  g_fisInput[1] = sensorData.light;      // Assuming humidity is related to light intensity
  g_fisInput[2] = sensorData.motion;     // Motion
  g_fisInput[3] = sensorData.gas_level;  // Gas Concentration
  g_fisInput[3] = sensorData.hour;       // Time of Day/Access Frequency
  g_fisInput[5] = sensorData.current;    // Energy Consumption

  // Evaluate the Fuzzy Inference System
  fis_evaluate();

  // Assign FIS outputs to actuator commands
  actuatorCommands[0] = g_fisOutput[0];  // Lights
  actuatorCommands[1] = g_fisOutput[1];  // Fan
  actuatorCommands[2] = g_fisOutput[2];  // Alarm
  actuatorCommands[3] = g_fisOutput[3];  // Energy Management Mode (or your 4th output)

  // ... (Assign other FIS outputs to actuatorCommands as needed) ...

  // Send actuator commands to Peripheral ESP32
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&actuatorCommands, sizeof(actuatorCommands));
  if (result == ESP_OK) {
    Serial.println("Sent actuator commands");
  } else {
    Serial.println("Error sending actuator commands");
  }
}

// Setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected at IP ");
  Serial.print(WiFi.localIP());


  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

// Loop  routine runs over and over again forever : 
void loop() {
  delay(10);  // Small delay to avoid overwhelming the ESP32
}

// ... (Rest of your FIS support functions: fis_trimf, fis_trapmf, etc. - unchanged) ...
//***********************************************************************
// Support functions for Fuzzy Inference System
//***********************************************************************
// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2 * (b <= x) * (x <= c));
    if (b == c) return (FIS_TYPE) (t1 * (a <= x) * (x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, (FIS_TYPE)0); // Explicitly cast 0 to FIS_TYPE
}

// Trapezoidal Member Function
// Trapezoidal Member Function
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
    return (FIS_TYPE) min(t1, t2);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return (FIS_TYPE) min(a, b); // Ensure consistent data types
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return (FIS_TYPE) max(a, b); // Ensure consistent data types
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
    fis_trimf, fis_trapmf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 3, 4, 2, 3, 3, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 4, 4, 3, 3 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 8.83333, 18, 22 };
FIS_TYPE fis_gMFI0Coeff2[] = { 21, 29, 35 };
FIS_TYPE fis_gMFI0Coeff3[] = { 30.8333333333333, 40, 49.1666666666667 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE fis_gMFI1Coeff1[] = { -284.444444444444, 0, 162.174757281553 };
FIS_TYPE fis_gMFI1Coeff2[] = { 56.8888888888889, 341.333333333333, 625.777777777778 };
FIS_TYPE fis_gMFI1Coeff3[] = { 398.222222222222, 682.666666666667, 967.111111111111 };
FIS_TYPE fis_gMFI1Coeff4[] = { 739.555555555556, 1024, 1308.44444444444 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3, fis_gMFI1Coeff4 };
FIS_TYPE fis_gMFI2Coeff1[] = { -0.833333333333333, 0, 0.833333333333333 };
FIS_TYPE fis_gMFI2Coeff2[] = { 0.166666666666667, 1, 1.83333333333333 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2 };
FIS_TYPE fis_gMFI3Coeff1[] = { -416.667, 0, 200 };
FIS_TYPE fis_gMFI3Coeff2[] = { 150, 350, 550 };
FIS_TYPE fis_gMFI3Coeff3[] = { 500, 1000, 1416.67 };
FIS_TYPE* fis_gMFI3Coeff[] = { fis_gMFI3Coeff1, fis_gMFI3Coeff2, fis_gMFI3Coeff3 };
FIS_TYPE fis_gMFI4Coeff1[] = { 2.468, 9.80257510729614, 17.5622317596567, 21.5793991416309 };
FIS_TYPE fis_gMFI4Coeff2[] = { 18.5236051502146, 21.0643776824034, 23.3648068669528 };
FIS_TYPE fis_gMFI4Coeff3[] = { 0, 2.24893, 3.75966 };
FIS_TYPE* fis_gMFI4Coeff[] = { fis_gMFI4Coeff1, fis_gMFI4Coeff2, fis_gMFI4Coeff3 };
FIS_TYPE fis_gMFI5Coeff1[] = { -41.6666666666667, 0, 41.6666666666667 };
FIS_TYPE fis_gMFI5Coeff2[] = { 8.33333333333334, 50, 91.6666666666667 };
FIS_TYPE fis_gMFI5Coeff3[] = { 58.3333333333333, 100, 141.666666666667 };
FIS_TYPE* fis_gMFI5Coeff[] = { fis_gMFI5Coeff1, fis_gMFI5Coeff2, fis_gMFI5Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff, fis_gMFI3Coeff, fis_gMFI4Coeff, fis_gMFI5Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { -70.8333333333333, 0, 6.01931330472103 };
FIS_TYPE fis_gMFO0Coeff2[] = { 2.37124463519313, 45.4184549356223, 96.8562231759657 };
FIS_TYPE fis_gMFO0Coeff3[] = { 60.3755364806867, 137.714592274678, 225.511444921316 };
FIS_TYPE fis_gMFO0Coeff4[] = { 211.770386266094, 255, 325.833333333333 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4 };
FIS_TYPE fis_gMFO1Coeff1[] = { -70.8333333333333, 0, 18.0579399141631 };
FIS_TYPE fis_gMFO1Coeff2[] = { 3.95207439198856, 74.7854077253219, 145.618741058655 };
FIS_TYPE fis_gMFO1Coeff3[] = { 90.7761087267525, 161.609442060086, 232.442775393419 };
FIS_TYPE fis_gMFO1Coeff4[] = { 184.166666666667, 255, 325.833333333333 };
FIS_TYPE* fis_gMFO1Coeff[] = { fis_gMFO1Coeff1, fis_gMFO1Coeff2, fis_gMFO1Coeff3, fis_gMFO1Coeff4 };
FIS_TYPE fis_gMFO2Coeff1[] = { -0.416666666666667, 0, 0.416666666666667 };
FIS_TYPE fis_gMFO2Coeff2[] = { 0.0833333333333333, 0.5, 0.916666666666667 };
FIS_TYPE fis_gMFO2Coeff3[] = { 0.583333333333333, 1, 1.41666666666667 };
FIS_TYPE* fis_gMFO2Coeff[] = { fis_gMFO2Coeff1, fis_gMFO2Coeff2, fis_gMFO2Coeff3 };
FIS_TYPE fis_gMFO3Coeff1[] = { -0.416666666666667, 0, 0.416666666666667 };
FIS_TYPE fis_gMFO3Coeff2[] = { 0.0833333333333333, 0.5, 0.916666666666667 };
FIS_TYPE fis_gMFO3Coeff3[] = { 0.583333333333333, 1, 1.41666666666667 };
FIS_TYPE* fis_gMFO3Coeff[] = { fis_gMFO3Coeff1, fis_gMFO3Coeff2, fis_gMFO3Coeff3 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff, fis_gMFO1Coeff, fis_gMFO2Coeff, fis_gMFO3Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0 };
int fis_gMFI1[] = { 0, 0, 0, 0 };
int fis_gMFI2[] = { 0, 0 };
int fis_gMFI3[] = { 0, 0, 0 };
int fis_gMFI4[] = { 1, 0, 0 };
int fis_gMFI5[] = { 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2, fis_gMFI3, fis_gMFI4, fis_gMFI5};

// Output membership function set
int fis_gMFO0[] = { 0, 0, 0, 0 };
int fis_gMFO1[] = { 0, 0, 0, 0 };
int fis_gMFO2[] = { 0, 0, 0 };
int fis_gMFO3[] = { 0, 0, 0 };
int* fis_gMFO[] = { fis_gMFO0, fis_gMFO1, fis_gMFO2, fis_gMFO3};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 3, 0, 0, 0, 0, 0 };
int fis_gRI1[] = { 2, 0, 0, 0, 0, 0 };
int fis_gRI2[] = { 1, 0, 0, 0, 0, 0 };
int fis_gRI3[] = { 1, 0, 0, 0, 0, 3 };
int fis_gRI4[] = { 2, 0, 0, 0, 0, 0 };
int fis_gRI5[] = { 0, 1, 2, 0, 0, 0 };
int fis_gRI6[] = { 0, 2, 2, 0, 0, 0 };
int fis_gRI7[] = { 0, 3, 2, 0, 1, 0 };
int fis_gRI8[] = { 0, 4, 2, 0, 0, 0 };
int fis_gRI9[] = { 0, 0, 1, 0, 0, 0 };
int fis_gRI10[] = { 0, 0, 0, 1, 0, 0 };
int fis_gRI11[] = { 0, 0, 0, 2, 0, 0 };
int fis_gRI12[] = { 0, 0, 0, 3, 0, 0 };
int fis_gRI13[] = { 0, 0, 0, 0, 1, 0 };
int fis_gRI14[] = { 0, 0, 0, 0, 2, 0 };
int fis_gRI15[] = { 0, 0, 0, 0, 3, 0 };
int fis_gRI16[] = { 0, 0, 0, 0, 0, 1 };
int fis_gRI17[] = { 0, 0, 0, 0, 0, 2 };
int fis_gRI18[] = { 0, 0, 0, 0, 0, 3 };
int fis_gRI19[] = { 0, 0, 0, 2, 2, 0 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13, fis_gRI14, fis_gRI15, fis_gRI16, fis_gRI17, fis_gRI18, fis_gRI19 };

// Rule Outputs
int fis_gRO0[] = { 1, 4, 0, 0 };
int fis_gRO1[] = { 0, 4, 0, 0 };
int fis_gRO2[] = { 0, 2, 0, 0 };
int fis_gRO3[] = { 0, 1, 0, 0 };
int fis_gRO4[] = { 0, 3, 0, 0 };
int fis_gRO5[] = { 4, 0, 0, 0 };
int fis_gRO6[] = { 4, 0, 0, 0 };
int fis_gRO7[] = { 4, 0, 0, 0 };
int fis_gRO8[] = { 2, 0, 0, 0 };
int fis_gRO9[] = { 1, 0, 0, 0 };
int fis_gRO10[] = { 0, 0, 1, 0 };
int fis_gRO11[] = { 0, 0, 2, 0 };
int fis_gRO12[] = { 0, 0, 3, 0 };
int fis_gRO13[] = { 0, 0, 1, 0 };
int fis_gRO14[] = { 1, 0, 2, 0 };
int fis_gRO15[] = { 0, 0, 3, 0 };
int fis_gRO16[] = { 0, 0, 0, 1 };
int fis_gRO17[] = { 0, 0, 0, 2 };
int fis_gRO18[] = { 0, 0, 0, 3 };
int fis_gRO19[] = { 0, 0, 3, 0 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13, fis_gRO14, fis_gRO15, fis_gRO16, fis_gRO17, fis_gRO18, fis_gRO19 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 18, 0, 0, 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 40, 1024, 1, 1000, 24, 100 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0, 0, 0, 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 255, 255, 1, 1 };

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
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0 };
    FIS_TYPE fuzzyInput3[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput4[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput5[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, fuzzyInput3, fuzzyInput4, fuzzyInput5, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0 };
    FIS_TYPE fuzzyOutput1[] = { 0, 0, 0, 0 };
    FIS_TYPE fuzzyOutput2[] = { 0, 0, 0 };
    FIS_TYPE fuzzyOutput3[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, fuzzyOutput1, fuzzyOutput2, fuzzyOutput3, };
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
