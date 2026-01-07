#include <iostream>

// Test function to simulate map ID validation
bool validateMapId_Old(int mapId) {
    if (mapId <= 0) {
        std::cerr << "Invalid map ID (OLD): " << mapId << std::endl;
        return false;
    }
    return true;
}

bool validateMapId_Fixed(int mapId) {
    if (mapId < 0) {
        std::cerr << "Invalid map ID (FIXED): " << mapId << std::endl;
        return false;
    }
    return true;
}

int main() {
    std::cout << "Testing Map ID Validation Logic" << std::endl;
    std::cout << "================================" << std::endl;
    
    int testMapIds[] = {-1, 0, 1, 2, 5};
    int numTests = sizeof(testMapIds) / sizeof(testMapIds[0]);
    
    for (int i = 0; i < numTests; i++) {
        int mapId = testMapIds[i];
        std::cout << "\nTesting Map ID: " << mapId << std::endl;
        
        std::cout << "  Old logic:   ";
        bool oldResult = validateMapId_Old(mapId);
        std::cout << (oldResult ? "VALID" : "INVALID") << std::endl;
        
        std::cout << "  Fixed logic: ";
        bool fixedResult = validateMapId_Fixed(mapId);
        std::cout << (fixedResult ? "VALID" : "INVALID") << std::endl;
        
        if (mapId == 0) {
            std::cout << "  >>> Map ID 0 should be VALID (this is the key fix!)" << std::endl;
            if (oldResult) {
                std::cout << "  >>> ERROR: Old logic incorrectly accepts map ID 0" << std::endl;
            } else {
                std::cout << "  >>> CONFIRMED: Old logic incorrectly rejects map ID 0" << std::endl;
            }
            
            if (fixedResult) {
                std::cout << "  >>> SUCCESS: Fixed logic correctly accepts map ID 0" << std::endl;
            } else {
                std::cout << "  >>> ERROR: Fixed logic still rejects map ID 0" << std::endl;
            }
        }
    }
    
    std::cout << "\n================================" << std::endl;
    std::cout << "Test completed!" << std::endl;
    
    return 0;
}
