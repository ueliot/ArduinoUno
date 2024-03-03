#include <Arduino.h>
#include <FreeRTOS.h>
#include <queue.h>

// Definir el tamaño de la cola para almacenar los datos
#define QUEUE_SIZE 10

// Declarar la cola para almacenar los datos
QueueHandle_t dataQueue;

// Declarar la tarea que enviará los datos por el puerto serie
void sendDataTask(void *parameter) {
  (void)parameter; // Evitar advertencia de parámetro no utilizado
  Serial.begin(9600);
  
  while (1) {
    if (uxQueueMessagesWaiting(dataQueue) > 0) {
      // Leer datos de la cola
      int data;
      xQueueReceive(dataQueue, &data, portMAX_DELAY);
      
      // Enviar datos por el puerto serie
      Serial.print("Dato recibido: ");
      Serial.println(data);
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Esperar un poco antes de verificar la cola nuevamente
  }
}

// Declarar la tarea que envía datos a la cola
void sendDataToQueueTask(void *parameter) {
  (void)parameter; // Evitar advertencia de parámetro no utilizado
  
  int counter = 0;
  
  while (1) {
    // Enviar datos a la cola
    xQueueSend(dataQueue, &counter, portMAX_DELAY);
    counter++;
    vTaskDelay(pdMS_TO_TICKS(500)); // Esperar un poco antes de enviar el siguiente dato
  }
}

void setup() {
  // Inicializar la cola para almacenar los datos
  dataQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
  
  // Crear la tarea para enviar los datos por el puerto serie
  xTaskCreatePinnedToCore(sendDataTask, "SendData", 2048, NULL, 1, NULL, 1);
  
  // Crear la tarea que envía datos a la cola
  xTaskCreatePinnedToCore(sendDataToQueueTask, "SendDataToQueue", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // No se hace nada en el bucle principal, las tareas se manejan en FreeRTOS
}

