#include <objectAlgo.h>
 


bool identifyArray(const int arr[], int size) {
    // Calculate differences between consecutive elements
    int differences[size - 1];
    for (int i = 1; i < size; ++i) {
        differences[i - 1] = arr[i] - arr[i - 1];
    }

    // Check if all differences are the same
    for (int i = 1; i < size - 1; ++i) {
        if (differences[i] != differences[i - 1]) {
            return false;
        }
    }

    return true;
}

void identifyObject() {
    // Measure distance
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH);
    
    // Calculate distance
    int distance = duration * 0.034 / 2;

    // Shift array and add new distance
    for (int i = arraySize - 1; i > 0; --i) {
        distanceArray[i] = distanceArray[i - 1];
    }
    distanceArray[0] = distance;

    // Check if array follows the pattern
    if (identifyArray(distanceArray, arraySize)) {
        // Turn on LED
        digitalWrite(ledPin, HIGH);
    } else {
        // Turn off LED
        digitalWrite(ledPin, LOW);
    }

    // Print distance for debugging
    Serial.print("Distance: ");
    Serial.println(distance);

    // Delay for stability
    delay(500);
}