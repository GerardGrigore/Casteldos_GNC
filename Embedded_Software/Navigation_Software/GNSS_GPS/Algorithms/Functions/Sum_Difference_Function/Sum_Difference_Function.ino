// Structure definition for tests. It is like defining the outputs of a function.
struct SUM_DIFFERENCE_LONG{
  long SUM;
  long DIFFERENCE;
};

// Then create the function:
SUM_DIFFERENCE_LONG FUNCTION_SUM_DIFFERENCE_CALCULATION(long INTEGER_1, long INTEGER_2){ // Define the inputs here.
  SUM_DIFFERENCE_LONG FUNCTION_SUM_DIFFERENCE;
  // Try to use temporary variables:
  long SUMATION = INTEGER_1 + INTEGER_2;
  long DIFFERENCIATION = INTEGER_1 - INTEGER_2;
  // Then define the output:
  FUNCTION_SUM_DIFFERENCE.SUM = SUMATION;
  FUNCTION_SUM_DIFFERENCE.DIFFERENCE = DIFFERENCIATION;
  // Then return the output:
  return FUNCTION_SUM_DIFFERENCE;
} 

void setup() {
  // Begin the serial monitor that will be used to print the data:
  Serial.begin(115200);
}

void loop() {
  // Call the previously defined function to test it:
  SUM_DIFFERENCE_LONG FUNCTION_SUM_DIFFERENCE = FUNCTION_SUM_DIFFERENCE_CALCULATION(10,2);
  Serial.print("SUM = ");
  Serial.print(F("/"));
  Serial.print(FUNCTION_SUM_DIFFERENCE.SUM);
  Serial.print(F("/"));
  Serial.print("DIFFERENCE = ");
  Serial.print(F("/"));
  Serial.print(FUNCTION_SUM_DIFFERENCE.DIFFERENCE);
  delay(5000);
}
