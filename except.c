#include "stm32f4xx.h"

// Function prototypes
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void triggerHardFault(void);
void triggerMemManageFault(void);
void triggerBusFault(void);
void triggerUsageFault(void);
void triggerSVCall(void);
void triggerPendSV(void);
void initializeSystem(void);
void configureMPU(void);
void toggleLED(void);

// Assuming an LED is connected to pin PC13
#define LED_PIN 13

// Main function
int main(void) {
    // Initialize system (e.g., configure clocks, peripherals)
    initializeSystem();
    configureMPU(); // Configure MPU for Memory Management Faults

    // Infinite loop
    while (1) {
        // Uncomment one of the following lines to test specific faults
        // triggerHardFault();
        // triggerMemManageFault();
        // triggerBusFault();
        // triggerUsageFault();
        // triggerSVCall();
        // triggerPendSV();
    }
}

// Configure the MPU to generate a Memory Management Fault
void configureMPU(void) {
    // Disable MPU before configuration
    MPU->CTRL = 0;

    // Configure MPU region for no access
    MPU->RNR = 0; // Region number 0
    MPU->RBAR = 0x20000000; // Base address (e.g., start of SRAM)
    MPU->RASR = (0x0 << MPU_RASR_AP_Pos)  // No access permission
                | (0x1F << MPU_RASR_SIZE_Pos) // Set region size (e.g., 512MB)
                | MPU_RASR_ENABLE_Msk; // Enable the region

    // Enable MPU with default memory map background region disabled
    MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;

    // Enable Memory Management Fault
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
}

// Function to trigger a Hard Fault
void triggerHardFault(void) {
    // Access an invalid address to trigger Hard Fault
    volatile int *pInvalidAddress = (int *)0xFFFFFFFF; // Invalid address
    *pInvalidAddress = 0; // Write to invalid address
}

// Function to trigger a Memory Management Fault
void triggerMemManageFault(void) {
    // Access the MPU-protected region to trigger a Memory Management Fault
    volatile int *pProtectedRegion = (int *)0x20000000; // Protected region
    *pProtectedRegion = 0; // Write to restricted memory region
}

// Function to trigger a Bus Fault
void triggerBusFault(void) {
    // Enable Bus Fault handler
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
    
    // Access an invalid peripheral address to trigger a Bus Fault
    volatile int *pInvalidPeriphAddress = (int *)0x60000000; // Invalid peripheral address
    *pInvalidPeriphAddress = 0; // Write to invalid peripheral space
}

// Function to trigger a Usage Fault
void triggerUsageFault(void) {
    // Enable Usage Fault
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;
    // Trigger an undefined instruction to cause Usage Fault
    __asm("UDF #0"); // Undefined instruction
}

// Function to trigger an SVC (Supervisor Call) Fault
void triggerSVCall(void) {
    __asm("SVC #0"); // Supervisor call to trigger SVC_Handler
}

// Function to trigger a PendSV Fault
void triggerPendSV(void) {
    // Set PendSV interrupt pending to trigger PendSV_Handler
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

// Hard Fault Handler
void HardFault_Handler(void) {
    while (1) {
        toggleLED(); // Toggle LED to indicate Hard Fault
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}

// Memory Management Fault Handler
void MemManage_Handler(void) {
    while (1) {
        toggleLED(); // Toggle LED to indicate Memory Management Fault
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}

// Bus Fault Handler
void BusFault_Handler(void) {
    while (1) {
        toggleLED(); // Toggle LED to indicate Bus Fault
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}

// Usage Fault Handler
void UsageFault_Handler(void) {
    while (1) {
        toggleLED(); // Toggle LED to indicate Usage Fault
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}

// SVCall Handler
void SVC_Handler(void) {
    while (1) {
        toggleLED(); // Toggle LED to indicate SVC Call
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}

// PendSV Handler
void PendSV_Handler(void) {
    while (1) {
        toggleLED(); // Toggle LED to indicate PendSV
        for (volatile int i = 0; i < 100000; i++); // Simple delay
    }
}

// SysTick Handler (for periodic interrupts)
void SysTick_Handler(void) {
    toggleLED(); // Toggle LED to indicate SysTick interrupt
}

// Example system initialization function
void initializeSystem(void) {
    // Enable the GPIO clock for port C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Configure PC13 as output (LED)
    GPIOC->MODER |= (1 << (LED_PIN * 2)); // Set mode to output
}

// Function to toggle the LED
void toggleLED(void) {
    GPIOC->ODR ^= (1 << LED_PIN); // Toggle PC13
}